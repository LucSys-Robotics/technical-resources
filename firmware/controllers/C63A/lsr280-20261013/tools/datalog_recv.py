#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LSR280 chassis data observation receiver
Compatible with Python 3.8.10 / Linux / CH343 USB serial

Usage:
    python3 datalog_recv.py 01
    python3 datalog_recv.py 01 --port /dev/ttyCH343USB0 --outdir ./logs

Name by data group number, the above example will generate:
    logs/01_data.csv  logs/01_event.csv  logs/01_stat.csv  logs/01_raw.txt

If any file in the group already exists, directly refuse to run to avoid overwriting collected long-term test data.
Add --force when remining is necessary.

Dependency:
    pip3 install pyserial
"""

import argparse
import csv
import math
import os
import re
import signal
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    sys.stderr.write("缺少 pyserial, 请先执行:  pip3 install pyserial\n")
    sys.exit(1)


# ----------------------------------------------------------------------------
# Column definitions consistent with firmware datalog_task.h and scaling factors
# When modifying firmware fields, you must update them simultaneously
# ----------------------------------------------------------------------------

SCALE_VEL = 1000.0   # mm/s      -> m/s
SCALE_OMG = 1000.0   # mrad/s    -> rad/s
SCALE_RPM = 10.0     # 0.1rpm    -> rpm
SCALE_CUR = 10.0     # 0.1A      -> A
SCALE_TMP = 10.0     # 0.1degC   -> degC
SCALE_VOL = 100.0    # 0.01V     -> V
SCALE_ANG = 100.0    # 0.01deg   -> deg
SCALE_SCL = 1000.0   # Per mille -> ratio

COUNTS_PER_REV = 16384.0
COUNTS_PER_DEG = COUNTS_PER_REV / 360.0

# (column name, scaling divisor) scaling to None means keeping the integer intact as is
D_FIELDS = [
    ("t_ms",   None), ("seq",    None), ("src",  None), ("en",   None),
    ("err",    None), ("hold",   None), ("park", None),
    ("scl",    SCALE_SCL), ("mprg", None),

    ("cmd_vx", SCALE_VEL), ("cmd_vy", SCALE_VEL), ("cmd_vz", SCALE_OMG),
    ("sm_vx",  SCALE_VEL), ("sm_vy",  SCALE_VEL), ("sm_vz",  SCALE_OMG),
    ("st_vx",  SCALE_VEL), ("st_vy",  SCALE_VEL), ("st_vz",  SCALE_OMG),
    ("fb_vx",  SCALE_VEL), ("fb_vy",  SCALE_VEL), ("fb_vz",  SCALE_OMG),

    ("st_tgt_A", None), ("st_tgt_B", None), ("st_tgt_C", None), ("st_tgt_D", None),
    ("st_act_A", None), ("st_act_B", None), ("st_act_C", None), ("st_act_D", None),

    ("rpm_tgt_A", SCALE_RPM), ("rpm_tgt_B", SCALE_RPM),
    ("rpm_tgt_C", SCALE_RPM), ("rpm_tgt_D", SCALE_RPM),
    ("spd_act_A", SCALE_VEL), ("spd_act_B", SCALE_VEL),
    ("spd_act_C", SCALE_VEL), ("spd_act_D", SCALE_VEL),

    ("cur_A", SCALE_CUR), ("cur_B", SCALE_CUR),
    ("cur_C", SCALE_CUR), ("cur_D", SCALE_CUR),
    ("tmp_A", SCALE_TMP), ("tmp_B", SCALE_TMP),
    ("tmp_C", SCALE_TMP), ("tmp_D", SCALE_TMP),

    ("vol",  SCALE_VOL), ("pitch", SCALE_ANG), ("roll", SCALE_ANG),
    ("yaw",  SCALE_ANG), ("gyro_z", SCALE_OMG),
    ("online_bm", None), ("err_bm", None), ("comm_qua", None), ("fw_drop", None),
    ("send_cnt", None),

    # AutoTest internal variables for heading control.
    # vz_min/vz_max/vz_flip are accumulated by the firmware over a 100ms window at 50Hz, not instantaneous sample values
    # The oscillation of the translation segment Vz can only be seen from the envelope width and number of flips; a 10Hz sample will be assorted directly when read.
    ("at_yaw0",   SCALE_ANG),   # Test coordinate system reference heading; after starting, it should remain constant
    ("at_yaw_err", SCALE_ANG),  # input for PD
    ("at_vz_raw", SCALE_OMG),   # PD output before limiting
    ("at_vz_min", SCALE_OMG),   # Send the lower envelope of Vz within the window
    ("at_vz_max", SCALE_OMG),   # Upper envelope of Vz is sent down within the window
    ("at_vz_flip", None),       # The number of times the Vz symbol flips in the window, a direct indicator of oscillations
    ("at_seg",    None),        # Current segment index
    ("at_state",  None),        # 0 = Standby 1 = Execute 2 = Pause between segments 3 = Finish
]

# Derived columns computed by the receiver (placed at the end of the CSV for direct plot output)
DERIVED_FIELDS = [
    "t_s",            # Seconds from the first frame, drawing with the horizontal axis
    "spd_mps",        # Composite linear velocity sqrt(fb_vx^2 + fb_vy^2)
    "slip_radps",     # Slip amount = fb_vz - gyro_z, the most critical diagnostic volume for four wheels and four turns
    "st_err_A_deg", "st_err_B_deg", "st_err_C_deg", "st_err_D_deg",  # Steering following error
    "mprg_deg",       # Maximum steering deviation (degrees)
    "cur_sum_A",      # Sum of the absolute currents of all four wheels, indicating total chassis load
    "send_rate",      # set_MotorPos times/second, directly reflecting CAN load dispatch
    "at_vz_span",     # Width of the Vz envelope inside the window = max-min, instantly showing the oscillation amplitude
    "host_ts",        # The host computer receives the time, used to verify the firmware timebase
]

D_HEADER = [name for name, _ in D_FIELDS] + DERIVED_FIELDS
E_HEADER = ["t_ms", "type", "old", "new", "host_ts"]
S_HEADER = ["t_ms", "uptime_s", "dist_m", "hold_cnt", "err_cnt",
            "cur_peak_A", "tmp_peak_C", "fw_drop", "host_ts"]

SUFFIXES = ("_data.csv", "_event.csv", "_stat.csv", "_raw.txt")


# Error code point definition, consistent with errCode_* in RobotControl_task.h
ERR_BITS = [
    (1 << 0,  "LowPower"),
    (1 << 1,  "StopKeyEn"),
    (1 << 2,  "SoftWareStop"),
    (1 << 3,  "Drv1_offline"),
    (1 << 4,  "Drv1_Err"),
    (1 << 5,  "Drv2_offline"),
    (1 << 6,  "Drv2_Err"),
    (1 << 7,  "Drv3_offline"),
    (1 << 8,  "Drv3_Err"),
    (1 << 9,  "Drv4_offline"),
    (1 << 10, "Drv4_Err"),
    (1 << 11, "SteeringHome"),
]

CMD_SOURCE = {
    0: "NONE", 1: "BootLoader", 2: "GamePad", 3: "RCJOY", 4: "APP",
    5: "ROS", 6: "CAN", 7: "Charger", 8: "AutoTest",
}


def decode_err(value):
    """Translate the ErrNum bitmap into a readable string"""
    try:
        v = int(value)
    except (TypeError, ValueError):
        return ""
    if v == 0:
        return "OK"
    names = [name for bit, name in ERR_BITS if v & bit]
    return "|".join(names) if names else "0x%X" % v


def check_group(group, outdir, force):
    """Validate the group name and check if it already exists. Returns the file prefix"""
    if not re.match(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$", group):
        sys.stderr.write(
            "组名 '%s' 不合法.\n"
            "只允许字母/数字/下划线/点/减号, 且必须以字母或数字开头. 例如: 01 02 endurance_a\n"
            % group)
        sys.exit(2)

    prefix = os.path.join(outdir, group)
    existing = [prefix + s for s in SUFFIXES if os.path.exists(prefix + s)]

    if existing and not force:
        sys.stderr.write("\n拒绝运行: 数据组 '%s' 已存在, 不能覆盖已采集的数据.\n\n" % group)
        for p in existing:
            try:
                size = os.path.getsize(p)
                mtime = datetime.fromtimestamp(os.path.getmtime(p)).strftime("%Y-%m-%d %H:%M:%S")
                sys.stderr.write("    %s   %d 字节   %s\n" % (p, size, mtime))
            except OSError:
                sys.stderr.write("    %s\n" % p)
        sys.stderr.write("\n请换一个组名, 或确认要重采后加 --force 覆盖.\n")
        sys.exit(3)

    if existing and force:
        sys.stderr.write("[警告] --force 生效, 即将覆盖数据组 '%s' 的 %d 个文件\n\n"
                         % (group, len(existing)))

    return prefix


class Session(object):
    """One acquisition session is responsible for writing 4 output files"""

    def __init__(self, prefix):
        self.prefix = prefix

        self.f_raw = open(prefix + "_raw.txt", "w", encoding="utf-8")
        self.f_data = open(prefix + "_data.csv", "w", newline="", encoding="utf-8")
        self.f_evt = open(prefix + "_event.csv", "w", newline="", encoding="utf-8")
        self.f_stat = open(prefix + "_stat.csv", "w", newline="", encoding="utf-8")

        self.w_data = csv.writer(self.f_data)
        self.w_evt = csv.writer(self.f_evt)
        self.w_stat = csv.writer(self.f_stat)

        self.w_data.writerow(D_HEADER)
        self.w_evt.writerow(E_HEADER)
        self.w_stat.writerow(S_HEADER)

        self.t0_ms = None          # Firmware timestamp of the first frame
        self.last_seq = None
        self.last_send = None      # The set_MotorPos count of the previous frame
        self.last_send_t = 0.0
        self.warned_fields = False # Field count mismatch only triggers one warning
        self.n_data = 0
        self.n_evt = 0
        self.n_stat = 0
        self.n_bad = 0
        self.n_lost = 0            # Estimated frame loss based on seq breaks
        self.last_flush = time.time()

    # ---------- Row Processing ----------

    def feed(self, line):
        self.f_raw.write(line + "\n")
        if not line:
            return
        kind = line[0]
        try:
            if kind == "D":
                self._on_data(line)
            elif kind == "E":
                self._on_event(line)
            elif kind == "S":
                self._on_stat(line)
            elif kind == "H":
                self._on_header(line)
            else:
                self.n_bad += 1
        except Exception:
            # Single-line parsing failures cannot affect long-term acquisition
            self.n_bad += 1

        if time.time() - self.last_flush > 2.0:
            self.flush()

    def _on_header(self, line):
        sys.stdout.write("[H] " + line + "\n")
        sys.stdout.flush()

    def _on_data(self, line):
        parts = line.split(",")
        # First field D followed by len(D_FIELDS) data fields
        if len(parts) != len(D_FIELDS) + 1:
            self.n_bad += 1
            # If the number of fields doesn't match, the entire round of data will be lost. You must shout it out immediately instead of counting silently
            if not self.warned_fields:
                self.warned_fields = True
                sys.stdout.write(
                    "\n*** 字段数不匹配, 数据行全部会被丢弃 ***\n"
                    "    固件发来 %d 个字段, 本脚本期望 %d 个\n"
                    "    固件与上位机脚本版本不一致, 请把 LUCSYS/tools/ 下的\n"
                    "    datalog_recv.py 重新拷到本机, 或核对 datalog_task.c 的 D 行字段\n"
                    "    原始数据仍完整保存在 _raw.txt, 更新脚本后可用 --replay 重新解析\n\n"
                    % (len(parts) - 1, len(D_FIELDS)))
                sys.stdout.flush()
            return

        raw = parts[1:]
        row = []
        val = {}
        for (name, scale), text in zip(D_FIELDS, raw):
            try:
                iv = int(text)
            except ValueError:
                self.n_bad += 1
                return
            v = iv / scale if scale else iv
            val[name] = v
            row.append(v)

        # Frame loss statistics
        seq = int(val["seq"])
        if self.last_seq is not None and seq > self.last_seq + 1:
            self.n_lost += seq - self.last_seq - 1
        self.last_seq = seq

        # Derived column
        if self.t0_ms is None:
            self.t0_ms = val["t_ms"]
        t_s = (val["t_ms"] - self.t0_ms) / 1000.0

        spd = math.sqrt(val["fb_vx"] ** 2 + val["fb_vy"] ** 2)
        slip = val["fb_vz"] - val["gyro_z"]
        st_err = []
        for ax in ("A", "B", "C", "D"):
            e_cnt = val["st_tgt_" + ax] - val["st_act_" + ax]
            st_err.append(e_cnt / COUNTS_PER_DEG)
        mprg_deg = val["mprg"] / COUNTS_PER_DEG
        cur_sum = sum(abs(val["cur_" + ax]) for ax in ("A", "B", "C", "D"))

        # set_MotorPos Delivery rate (times/second), obtained from the cumulative count difference
        send_rate = 0.0
        if self.last_send is not None and t_s > self.last_send_t:
            send_rate = (val["send_cnt"] - self.last_send) / (t_s - self.last_send_t)
        self.last_send = val["send_cnt"]
        self.last_send_t = t_s

        vz_span = val["at_vz_max"] - val["at_vz_min"]

        row.extend([
            round(t_s, 3), round(spd, 4), round(slip, 4),
            round(st_err[0], 3), round(st_err[1], 3),
            round(st_err[2], 3), round(st_err[3], 3),
            round(mprg_deg, 3), round(cur_sum, 2), round(send_rate, 1),
            round(vz_span, 4),
            datetime.now().strftime("%H:%M:%S.%f")[:-3],
        ])

        self.w_data.writerow(row)
        self.n_data += 1

    def _on_event(self, line):
        parts = line.split(",")
        if len(parts) < 5:
            self.n_bad += 1
            return
        t_ms, etype, oldv, newv = parts[1], parts[2], parts[3], parts[4]
        host = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.w_evt.writerow([t_ms, etype, oldv, newv, host])
        self.n_evt += 1

        # Events directly printed to the terminal, visible on site immediately
        note = ""
        try:
            if etype == "ERR":
                note = "  %s -> %s" % (decode_err(oldv), decode_err(newv))
            elif etype == "SRC":
                note = "  %s -> %s" % (CMD_SOURCE.get(int(oldv), oldv),
                                       CMD_SOURCE.get(int(newv), newv))
        except (TypeError, ValueError):
            note = ""
        sys.stdout.write("[EVT %8s ms] %-7s %s -> %s%s\n"
                         % (t_ms, etype, oldv, newv, note))
        sys.stdout.flush()
        self.flush()

    def _on_stat(self, line):
        parts = line.split(",")
        if len(parts) < 8:
            self.n_bad += 1
            return
        t_ms = parts[1]
        uptime = parts[2]
        dist = int(parts[3]) / 100.0
        hold_cnt = parts[4]
        err_cnt = parts[5]
        cur_peak = int(parts[6]) / SCALE_CUR
        tmp_peak = int(parts[7]) / SCALE_TMP
        drop = parts[8] if len(parts) > 8 else "0"
        host = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.w_stat.writerow([t_ms, uptime, dist, hold_cnt, err_cnt,
                              cur_peak, tmp_peak, drop, host])
        self.n_stat += 1

    # ---------- Maintenance ----------

    def flush(self):
        self.f_raw.flush()
        self.f_data.flush()
        self.f_evt.flush()
        self.f_stat.flush()
        os.fsync(self.f_data.fileno())
        self.last_flush = time.time()

    def close(self):
        self.flush()
        for f in (self.f_raw, self.f_data, self.f_evt, self.f_stat):
            try:
                f.close()
            except Exception:
                pass

    def summary(self):
        return ("数据行=%d  事件=%d  统计=%d  丢帧(按seq)=%d  坏行=%d"
                % (self.n_data, self.n_evt, self.n_stat, self.n_lost, self.n_bad))


_running = True


def _on_sigint(signum, frame):
    global _running
    _running = False


def main():
    ap = argparse.ArgumentParser(description="T280 底盘数据观察接收端")
    ap.add_argument("group",
                    help="数据组编号, 如 01. 生成 <组名>_data.csv 等4个文件")
    ap.add_argument("--port", default="/dev/ttyCH343USB0", help="串口设备")
    ap.add_argument("--baud", type=int, default=115200, help="波特率")
    ap.add_argument("--outdir", default="./logs", help="输出目录")
    ap.add_argument("--force", action="store_true",
                    help="允许覆盖同名数据组(默认拒绝, 防止误删长测数据)")
    ap.add_argument("--replay", metavar="RAW_TXT", default=None,
                    help="离线重解析已有的 *_raw.txt, 不连串口. "
                         "字段数对不上导致 data.csv 为空时用它补救")
    args = ap.parse_args()

    if not os.path.isdir(args.outdir):
        os.makedirs(args.outdir)

    prefix = check_group(args.group, args.outdir, args.force)

    signal.signal(signal.SIGINT, _on_sigint)

    # ---- Offline Re-parsing Mode ----
    if args.replay:
        if not os.path.isfile(args.replay):
            sys.stderr.write("找不到原始文件: %s\n" % args.replay)
            sys.exit(2)
        sess = Session(prefix)
        sys.stdout.write("离线重解析: %s -> %s_*\n\n" % (args.replay, prefix))
        with open(args.replay, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                line = line.strip("\r\n \t")
                if line:
                    sess.feed(line)
        sess.close()
        sys.stdout.write("\n重解析完成. %s\n" % sess.summary())
        if sess.n_data == 0:
            sys.stdout.write(
                "\n[警告] 数据行仍为 0. 本脚本期望每行 %d 个字段(含行首D),\n"
                "       请检查固件的 datalog_task.c 与本脚本的 D_FIELDS 是否同步.\n"
                % (len(D_FIELDS) + 1))
        else:
            sys.stdout.write("出图:  python3 datalog_plot.py %s\n" % args.group)
        return

    sess = Session(prefix)
    sys.stdout.write("数据组: %s\n" % args.group)
    sys.stdout.write("输出文件: %s_data.csv / _event.csv / _stat.csv / _raw.txt\n" % prefix)
    sys.stdout.write("串口: %s @ %d   Ctrl+C 结束\n\n" % (args.port, args.baud))
    sys.stdout.flush()

    ser = None
    buf = b""
    last_report = time.time()
    last_n = 0

    try:
        while _running:
            # ---- Connect / Reconnect ----
            if ser is None:
                try:
                    ser = serial.Serial(args.port, args.baud, timeout=0.2)
                    ser.reset_input_buffer()
                    sys.stdout.write("已连接 %s\n" % args.port)
                    sys.stdout.flush()
                except Exception as e:
                    sys.stdout.write("打开串口失败(%s), 2秒后重试...\n" % e)
                    sys.stdout.flush()
                    time.sleep(2.0)
                    continue

            # ---- Read ----
            try:
                chunk = ser.read(4096)
            except Exception as e:
                sys.stdout.write("串口读取异常(%s), 重连中...\n" % e)
                sys.stdout.flush()
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(1.0)
                continue

            if chunk:
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("utf-8", errors="replace").strip("\r\n \t")
                    if line:
                        sess.feed(line)
                # Prevent unbounded buffer growth under abnormal conditions
                if len(buf) > 65536:
                    buf = b""
                    sess.n_bad += 1

            # ---- Status every 5 seconds----
            now = time.time()
            if now - last_report >= 5.0:
                rate = (sess.n_data - last_n) / (now - last_report)
                sys.stdout.write("  [%s] %s  实测速率=%.1f Hz\n"
                                 % (datetime.now().strftime("%H:%M:%S"),
                                    sess.summary(), rate))
                sys.stdout.flush()
                last_report = now
                last_n = sess.n_data
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        sess.close()
        sys.stdout.write("\n采集结束. 数据组 '%s'  %s\n" % (args.group, sess.summary()))
        sys.stdout.write("出图:  python3 datalog_plot.py %s\n" % args.group)


if __name__ == "__main__":
    main()
