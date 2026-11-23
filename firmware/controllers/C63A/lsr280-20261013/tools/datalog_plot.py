#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LSR280 chassis data observation - drawing script
Compatible with Python 3.8.10

Usage:
    python3 datalog_plot.py 01
    python3 datalog_plot.py 01 --decimate 20 --logdir ./logs --outdir ./charts

Retrieve logs/01_data.csv by data group number, output images to charts/01/
You can also directly provide the CSV path: python3 datalog_plot.py logs/01_data.csv

Automatically finds _event.csv within the same group and overlays fault events as vertical lines onto related diagrams.

Dependency:
    pip3 install pandas matplotlib
Chinese titles require CJK font; missing ones will automatically revert to English:
    sudo apt install fonts-noto-cjk && rm -rf ~/.cache/matplotlib
"""

import argparse
import os
import sys

try:
    import pandas as pd
except ImportError:
    sys.stderr.write("缺少 pandas, 请先执行:  pip3 install pandas\n")
    sys.exit(1)

try:
    import matplotlib
    matplotlib.use("Agg")          # Images can be produced even without a display environment
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
except ImportError:
    sys.stderr.write("缺少 matplotlib, 请先执行:  pip3 install matplotlib\n")
    sys.exit(1)


# ----------------------------------------------------------------------------
# Chinese fonts: On Linux, matplotlib's default font does not include Chinese fonts; if not set, it will display as a block.
# When Chinese fonts cannot be found, the English title is automatically returned to ensure the image is readable.
# ----------------------------------------------------------------------------
_CJK_CANDIDATES = [
    "Noto Sans CJK SC", "Noto Sans CJK JP", "Source Han Sans SC",
    "WenQuanYi Micro Hei", "WenQuanYi Zen Hei", "Microsoft YaHei", "SimHei",
]


def _setup_font():
    available = set(f.name for f in font_manager.fontManager.ttflist)
    matplotlib.rcParams["axes.unicode_minus"] = False
    for name in _CJK_CANDIDATES:
        if name in available:
            matplotlib.rcParams["font.sans-serif"] = [name]
            return True
    sys.stderr.write(
        "[提示] 未找到中文字体, 图表改用英文标题.\n"
        "       需要中文时执行:  sudo apt install fonts-noto-cjk\n"
        "       然后清缓存:      rm -rf ~/.cache/matplotlib\n\n")
    return False


USE_CN = _setup_font()


def T(cn, en):
    """Choose Chinese or English copy based on font availability"""
    return cn if USE_CN else en


WHEELS = ("A", "B", "C", "D")
EVENT_COLOR = {
    "ERR": "red", "EN": "orange", "HOLD": "purple",
    "ONLINE": "brown", "PARK": "gray", "SRC": "green",
}


def load_events(data_path):
    evt_path = data_path.replace("_data.csv", "_event.csv")
    if not os.path.isfile(evt_path):
        return None
    try:
        ev = pd.read_csv(evt_path)
        return ev if len(ev) else None
    except Exception:
        return None


def mark_events(ax, events, t0_ms):
    """Draw events as vertical lines, marking only the categories you truly care about to avoid overcrowding the picture"""
    if events is None:
        return
    shown = set()
    for _, r in events.iterrows():
        etype = str(r.get("type", ""))
        if etype not in ("ERR", "EN", "ONLINE"):
            continue
        try:
            t = (float(r["t_ms"]) - t0_ms) / 1000.0
        except (TypeError, ValueError):
            continue
        ax.axvline(t, color=EVENT_COLOR.get(etype, "black"),
                   linestyle="--", linewidth=0.8, alpha=0.7,
                   label=etype if etype not in shown else None)
        shown.add(etype)


def save(fig, outdir, name):
    path = os.path.join(outdir, name)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print("  " + path)


def resolve_group(group, logdir, outdir):
    """Parse the group number to (data file path, image output directory). Also accept direct CSV paths"""
    if group.endswith(".csv"):
        data_path = group
        name = os.path.basename(group)
        if name.endswith("_data.csv"):
            name = name[:-len("_data.csv")]
    else:
        data_path = os.path.join(logdir, group + "_data.csv")
        name = group

    if not os.path.isfile(data_path):
        sys.stderr.write("找不到数据文件: %s\n\n" % data_path)
        if os.path.isdir(logdir):
            groups = sorted(f[:-len("_data.csv")] for f in os.listdir(logdir)
                            if f.endswith("_data.csv"))
            if groups:
                sys.stderr.write("%s 下已有的数据组:\n" % logdir)
                for g in groups:
                    sys.stderr.write("    %s\n" % g)
            else:
                sys.stderr.write("%s 下没有任何数据组\n" % logdir)
        else:
            sys.stderr.write("目录 %s 不存在\n" % logdir)
        sys.exit(2)

    return data_path, os.path.join(outdir, name), name


def main():
    ap = argparse.ArgumentParser(description="LSR280 data observation plots")
    ap.add_argument("group",
                    help="数据组编号, 如 01; 也可直接给 *_data.csv 路径")
    ap.add_argument("--logdir", default="./logs", help="数据目录")
    ap.add_argument("--outdir", default="./charts", help="图片输出根目录")
    ap.add_argument("--decimate", type=int, default=1,
                    help="抽稀倍数, 长时间数据建议 10~50, 否则 matplotlib 会很慢")
    args = ap.parse_args()

    data_path, chart_dir, name = resolve_group(args.group, args.logdir, args.outdir)

    if not os.path.isdir(chart_dir):
        os.makedirs(chart_dir)

    df = pd.read_csv(data_path)
    if len(df) == 0:
        sys.stderr.write("数据为空: %s\n" % data_path)
        sys.exit(1)

    events = load_events(data_path)
    t0_ms = float(df["t_ms"].iloc[0])

    if args.decimate > 1:
        df = df.iloc[::args.decimate].copy()

    t = df["t_s"]
    dur_h = (t.iloc[-1] - t.iloc[0]) / 3600.0
    print("数据组 '%s'   源文件 %s" % (name, data_path))
    print("样本 %d 条, 时长 %.2f 小时, 抽稀 %dx" % (len(df), dur_h, args.decimate))
    print("输出:")

    x_lab = T("时间 (s)", "Time (s)")
    wl = T("轮 ", "Wheel ")

    # ---- 1. Temperature Trend ----
    fig, ax = plt.subplots(figsize=(14, 5))
    for w in WHEELS:
        ax.plot(t, df["tmp_" + w], label=wl + w, linewidth=1)
    ax.axhline(120, color="red", linestyle=":",
               label=T("驱动器保护阈值 120C", "Driver limit 120C"))
    mark_events(ax, events, t0_ms)
    ax.set_xlabel(x_lab)
    ax.set_ylabel(T("温度 (degC)", "Temperature (degC)"))
    ax.set_title(T("轮毂电机温度趋势 - 看是否收敛, 以及四轮温差",
                   "Hub motor temperature - convergence and wheel-to-wheel spread"))
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    save(fig, chart_dir, "01_temperature.png")

    # ---- 2. Current trend + slope ----
    fig, ax = plt.subplots(figsize=(14, 5))
    for w in WHEELS:
        ax.plot(t, df["cur_" + w], label=wl + w, linewidth=0.8)
    ax.set_xlabel(x_lab)
    ax.set_ylabel(T("电流 (A)", "Current (A)"))
    ax.set_title(T("轮毂电机电流 - 单轮持续偏高=机械/胎压/编码器问题; 尖峰对齐pitch=上坡",
                   "Hub motor current - one wheel high = mechanical issue; "
                   "spikes aligned with pitch = slope"))
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    ax2 = ax.twinx()
    ax2.plot(t, df["pitch"], color="gray", alpha=0.45, linewidth=0.8, label="pitch")
    ax2.set_ylabel("pitch (deg)")
    ax2.legend(loc="upper right", fontsize=8)
    mark_events(ax, events, t0_ms)
    save(fig, chart_dir, "02_current.png")

    # ---- 3. Steering Follow Error + Speed Limit Ratio ----
    fig, ax = plt.subplots(figsize=(14, 5))
    for w in WHEELS:
        ax.plot(t, df["st_err_%s_deg" % w], label=wl + w, linewidth=0.8)
    ax.set_xlabel(x_lab)
    ax.set_ylabel(T("转向跟随误差 (deg)", "Steering following error (deg)"))
    ax.set_title(T("转向跟随误差 - 误差大时 scl 应同步下降, 未下降说明限速逻辑未生效",
                   "Steering error - scl should drop when error grows; "
                   "if not, the speed limiter is not working"))
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    ax2 = ax.twinx()
    ax2.plot(t, df["scl"], color="black", alpha=0.5, linewidth=0.9, label="steer_scale")
    ax2.set_ylabel(T("限速比例", "steer_scale"))
    ax2.set_ylim(-0.05, 1.05)
    ax2.legend(loc="upper right", fontsize=8)
    save(fig, chart_dir, "03_steer_error.png")

    # ---- 4. Slip amount + deceleration hold gate ----
    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(t, df["slip_radps"], color="crimson", linewidth=0.8,
            label=T("打滑量 fb_vz - gyro_z", "Slip = fb_vz - gyro_z"))
    ax.plot(t, df["fb_vz"], color="steelblue", alpha=0.5, linewidth=0.7,
            label=T("轮速正解 fb_vz", "Kinematic fb_vz"))
    ax.plot(t, df["gyro_z"], color="seagreen", alpha=0.5, linewidth=0.7,
            label=T("IMU gyro_z", "IMU gyro_z"))
    ax.set_xlabel(x_lab)
    ax.set_ylabel(T("角速度 (rad/s)", "Angular rate (rad/s)"))
    ax.set_title(T("打滑量 - 尖峰对齐hold=泄压闸不够快; 不对齐hold=有工况漏判",
                   "Slip - spikes aligned with hold = gate too slow; "
                   "not aligned = missed condition"))
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    ax2 = ax.twinx()
    ax2.fill_between(t, 0, df["hold"], color="purple", alpha=0.2, step="post")
    ax2.set_ylabel("ReorientHold")
    ax2.set_ylim(-0.1, 1.5)
    save(fig, chart_dir, "04_slip.png")

    # ---- 5. Instruction Tracking Chain ----
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    has_steer = "st_vz" in df.columns
    for i, (axis, unit) in enumerate((("vx", "m/s"), ("vy", "m/s"), ("vz", "rad/s"))):
        a = axes[i]
        a.plot(t, df["cmd_" + axis], label=T("cmd 原始指令", "cmd (raw)"), linewidth=0.9)
        if has_steer:
            a.plot(t, df["st_" + axis], label=T("st 转向平滑后", "st (steering)"),
                   linewidth=1.1, linestyle="--")
        a.plot(t, df["sm_" + axis], label=T("sm 驱动平滑后", "sm (drive)"), linewidth=0.9)
        a.plot(t, df["fb_" + axis], label=T("fb 正解实际", "fb (actual)"), linewidth=0.9)
        a.set_ylabel("%s (%s)" % (axis, unit))
        a.grid(alpha=0.3)
        a.legend(loc="upper left", fontsize=8)
    axes[0].set_title(T("指令跟随链 - cmd与st的差=转向平滑挡掉的抖动; st决定轮角, sm决定轮速",
                        "Command chain - cmd vs st = steering filter; "
                        "st sets wheel angle, sm sets wheel speed"))
    axes[-1].set_xlabel(x_lab)
    save(fig, chart_dir, "05_command_chain.png")

    # ---- 6. Voltage vs. Total Current ----
    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(t, df["vol"], color="darkblue", linewidth=1,
            label=T("电池电压", "Battery voltage"))
    ax.axhline(21.5, color="green", linestyle=":", linewidth=0.9,
               label=T("低电恢复 21.5V", "Recover 21.5V"))
    ax.axhline(20.5, color="orange", linestyle=":", linewidth=0.9,
               label=T("低电告警 20.5V", "Warning 20.5V"))
    ax.axhline(20.0, color="red", linestyle=":", linewidth=0.9,
               label=T("失能阈值 20.0V", "Disable 20.0V"))
    ax.set_xlabel(x_lab)
    ax.set_ylabel(T("电压 (V)", "Voltage (V)"))
    ax.set_title(T("电压与总电流 - 验证 LowPowerFlag/en_flag 的动作点是否与阈值一致",
                   "Voltage vs total current - verify LowPowerFlag/en_flag trip points"))
    ax.grid(alpha=0.3)
    ax.legend(loc="lower left", fontsize=8)
    ax2 = ax.twinx()
    ax2.plot(t, df["cur_sum_A"], color="darkorange", alpha=0.55, linewidth=0.8,
             label=T("四轮电流和", "Total current"))
    ax2.set_ylabel(T("总电流 (A)", "Total current (A)"))
    ax2.legend(loc="lower right", fontsize=8)
    mark_events(ax, events, t0_ms)
    save(fig, chart_dir, "06_power.png")

    # ---- 7. Run Overview ----
    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(t, df["spd_mps"], color="teal", linewidth=0.8,
            label=T("合成线速度", "Speed"))
    ax.set_xlabel(x_lab)
    ax.set_ylabel(T("速度 (m/s)", "Speed (m/s)"))
    ax.set_title(T("运行总览 - 速度/使能/报错时间线",
                   "Overview - speed / enable / error timeline"))
    ax.grid(alpha=0.3)
    ax2 = ax.twinx()
    ax2.fill_between(t, 0, df["en"], color="green", alpha=0.15, step="post")
    ax2.plot(t, (df["err"] > 0).astype(int), color="red", linewidth=0.9,
             label=T("有报错", "Error active"))
    ax2.set_ylim(-0.1, 1.5)
    ax2.set_ylabel("en / err")
    ax2.legend(loc="upper right", fontsize=8)
    ax.legend(loc="upper left", fontsize=8)
    save(fig, chart_dir, "07_overview.png")

    # ---- Summary ----
    print("\n关键统计:")
    print("  运行时长      : %.2f 小时" % dur_h)
    hottest = max(WHEELS, key=lambda w: df["tmp_" + w].max())
    print("  最高温度      : %.1f degC (轮 %s)" % (df["tmp_" + hottest].max(), hottest))
    print("  电流绝对值峰值: %.2f A" % max(df["cur_" + w].abs().max() for w in WHEELS))
    print("  转向误差峰值  : %.2f deg"
          % max(df["st_err_%s_deg" % w].abs().max() for w in WHEELS))
    print("  打滑量峰值    : %.3f rad/s" % df["slip_radps"].abs().max())
    print("  电压区间      : %.2f ~ %.2f V" % (df["vol"].min(), df["vol"].max()))
    print("  固件丢帧计数  : %d" % df["fw_drop"].max())
    if events is not None:
        print("  事件条数      : %d" % len(events))
        vc = events["type"].value_counts()
        for k in vc.index:
            print("      %-8s %d" % (k, vc[k]))


if __name__ == "__main__":
    main()
