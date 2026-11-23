#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
T280 数据观察 - 两组数据对比
适配 Python 3.8.10

用法:
    python3 datalog_compare.py 04 05
    python3 datalog_compare.py 04 05 --logdir ./logs

把"改动前"和"改动后"两组数据放在一起, 逐项算出改善倍数.
判据内置在脚本里(哪些指标该降、哪些不能变差), 直接给出结论.

依赖:
    pip3 install pandas
"""

import argparse
import os
import sys

try:
    import pandas as pd
except ImportError:
    sys.stderr.write("缺少 pandas, 请先执行:  pip3 install pandas\n")
    sys.exit(1)


COUNTS_PER_DEG = 16384.0 / 360.0
WHEELS = ("A", "B", "C", "D")


def load(group, logdir):
    path = group if group.endswith(".csv") else os.path.join(logdir, group + "_data.csv")
    if not os.path.isfile(path):
        sys.stderr.write("找不到: %s\n" % path)
        sys.exit(2)
    df = pd.read_csv(path)
    evt_path = path.replace("_data.csv", "_event.csv")
    ev = None
    if os.path.isfile(evt_path):
        try:
            ev = pd.read_csv(evt_path)
        except Exception:
            ev = None
    return df, ev, path


def metrics(df, ev):
    """提取用于对比的指标"""
    m = {}
    n = len(df)
    m["_n"] = n
    m["时长_分钟"] = (df["t_s"].iloc[-1] - df["t_s"].iloc[0]) / 60.0

    # ---- 转向目标跳变: 本次改动的直接目标 ----
    jump_deg = None
    for w in WHEELS:
        d = df["st_tgt_" + w].diff().abs() / COUNTS_PER_DEG
        jump_deg = d if jump_deg is None else pd.concat([jump_deg, d], axis=1).max(axis=1)
    jump_deg = jump_deg.fillna(0)
    m["目标跳变>50度_次数"] = int((jump_deg > 50).sum())
    m["目标跳变>20度_次数"] = int((jump_deg > 20).sum())
    m["目标跳变_峰值度"] = float(jump_deg.max())

    # ---- 转向误差 ----
    err = None
    for w in WHEELS:
        e = df["st_err_%s_deg" % w].abs()
        err = e if err is None else pd.concat([err, e], axis=1).max(axis=1)
    m["转向误差_峰值度"] = float(err.max())
    m["转向误差_均值度"] = float(err.mean())
    m["转向误差>90度_占比%"] = 100.0 * (err > 90).sum() / n

    # ---- CAN 下发负载 ----
    if "send_rate" in df.columns:
        m["下发速率_均值_次每秒"] = float(df["send_rate"].mean())
        m["下发速率_峰值_次每秒"] = float(df["send_rate"].max())
    # comm_qua 只看均值与低分位占比. 最小值是离群点, 不能用来判断总线好坏:
    # 它是比值(100-失败/总SDO), 分母随下发量变化, 且计数器每10秒硬清零,
    # 窗口末尾必然最难看. 实测正常状态均值 98%+, 低于70%的样本万分之一量级.
    m["comm_qua_均值%"] = float(df["comm_qua"].mean())
    m["comm_qua_低于95占比%"] = 100.0 * (df["comm_qua"] < 95).sum() / n
    m["comm_qua_最低%_仅参考"] = float(df["comm_qua"].min())
    m["固件丢帧_fw_drop"] = int(df["fw_drop"].max())

    # ---- 指令抖动 (上游, 本次改动不负责, 用于确认输入可比) ----
    m["cmd_vz打满±0.30_占比%"] = 100.0 * (df["cmd_vz"].abs() >= 0.299).sum() / n
    sign = df["cmd_vz"].apply(lambda v: 1 if v > 0.1 else (-1 if v < -0.1 else 0))
    m["cmd_vz正负翻转_次数"] = int((sign.diff().abs() == 2).sum())

    # ---- 转向平滑器效果 (新增列, 老数据没有) ----
    if "st_vz" in df.columns:
        m["st_vz打满±0.30_占比%"] = 100.0 * (df["st_vz"].abs() >= 0.299).sum() / n
        s2 = df["st_vz"].apply(lambda v: 1 if v > 0.1 else (-1 if v < -0.1 else 0))
        m["st_vz正负翻转_次数"] = int((s2.diff().abs() == 2).sum())

    # ---- 不能变差的项 ----
    m["scl_0与1之间跳变_次数"] = int((df["scl"].diff().abs() > 0.5).sum())
    m["最高速度_m每s"] = float(df["spd_mps"].max())
    m["温度峰值_C"] = float(max(df["tmp_" + w].max() for w in WHEELS))
    m["电流峰值_A"] = float(max(df["cur_" + w].abs().max() for w in WHEELS))
    m["驱动掉线_行数"] = int((df["online_bm"] != 15).sum())
    m["驱动报错_行数"] = int((df["err_bm"] != 0).sum())

    # ---- 事件 ----
    m["HOLD事件_次数"] = 0
    m["ERR事件_次数"] = 0
    if ev is not None and "type" in ev.columns:
        vc = ev["type"].value_counts()
        m["HOLD事件_次数"] = int(vc.get("HOLD", 0))
        m["ERR事件_次数"] = int(vc.get("ERR", 0))
    return m


# (指标名, 方向)  方向: down=越小越好  up=越大越好  same=不应明显变化  info=仅参考
JUDGE = [
    ("目标跳变>50度_次数",      "down"),
    ("目标跳变>20度_次数",      "down"),
    ("目标跳变_峰值度",         "down"),
    ("转向误差_峰值度",         "down"),
    ("转向误差_均值度",         "down"),
    ("转向误差>90度_占比%",     "down"),
    ("下发速率_均值_次每秒",    "down"),
    ("下发速率_峰值_次每秒",    "down"),
    ("comm_qua_均值%",          "up"),
    ("comm_qua_低于95占比%",    "down"),
    ("comm_qua_最低%_仅参考",   "info"),
    ("固件丢帧_fw_drop",        "down"),
    ("scl_0与1之间跳变_次数",   "down"),
    ("cmd_vz打满±0.30_占比%",   "info"),
    ("cmd_vz正负翻转_次数",     "info"),
    ("st_vz打满±0.30_占比%",    "info"),
    ("st_vz正负翻转_次数",      "info"),
    ("最高速度_m每s",           "same"),
    ("温度峰值_C",              "info"),
    ("电流峰值_A",              "info"),
    ("驱动掉线_行数",           "down"),
    ("驱动报错_行数",           "down"),
    ("HOLD事件_次数",           "same"),
    ("ERR事件_次数",            "info"),
    ("时长_分钟",               "info"),
]


def fmt(v):
    if isinstance(v, float):
        return "%.2f" % v
    return str(v)


def main():
    ap = argparse.ArgumentParser(description="T280 两组数据对比")
    ap.add_argument("before", help="改动前的数据组, 如 04")
    ap.add_argument("after", help="改动后的数据组, 如 05")
    ap.add_argument("--logdir", default="./logs")
    args = ap.parse_args()

    df1, ev1, p1 = load(args.before, args.logdir)
    df2, ev2, p2 = load(args.after, args.logdir)
    m1 = metrics(df1, ev1)
    m2 = metrics(df2, ev2)

    print("改动前: %s  (%d 行)" % (p1, m1["_n"]))
    print("改动后: %s  (%d 行)" % (p2, m2["_n"]))
    print()
    print("%-26s %12s %12s %12s  %s" % ("指标", "改动前", "改动后", "变化", "判定"))
    print("-" * 82)

    verdict_good, verdict_bad = [], []

    for name, direction in JUDGE:
        v1 = m1.get(name)
        v2 = m2.get(name)
        if v1 is None and v2 is None:
            continue
        if v1 is None:
            print("%-26s %12s %12s %12s  %s" % (name, "n/a", fmt(v2), "-", "新增指标"))
            continue
        if v2 is None:
            continue

        # 变化描述
        if v1 == 0 and v2 == 0:
            change, tag = "0 -> 0", ""
        elif v1 == 0:
            change, tag = "0 -> %s" % fmt(v2), ""
        else:
            ratio = v2 / v1 if v1 else 0
            change = "%.2fx" % ratio
            tag = ""

        if direction == "down" and v1 > 0:
            if v2 <= v1 * 0.5:
                tag = "明显改善"
                verdict_good.append(name)
            elif v2 <= v1 * 0.9:
                tag = "改善"
            elif v2 >= v1 * 1.1:
                tag = "变差 <<<"
                verdict_bad.append(name)
            else:
                tag = "基本持平"
        elif direction == "up" and v1 > 0:
            if v2 >= v1 * 1.05:
                tag = "改善"
                verdict_good.append(name)
            elif v2 <= v1 * 0.95:
                tag = "变差 <<<"
                verdict_bad.append(name)
            else:
                tag = "基本持平"
        elif direction == "same":
            if v1 > 0 and (v2 < v1 * 0.7 or v2 > v1 * 1.4):
                tag = "偏离 <<< 需人工确认"
                verdict_bad.append(name)
            else:
                tag = "正常"
        else:
            tag = "参考"

        print("%-26s %12s %12s %12s  %s" % (name, fmt(v1), fmt(v2), change, tag))

    print()
    print("=" * 82)
    if verdict_bad:
        print("需要人工确认的项:")
        for k in verdict_bad:
            print("   - %s" % k)
    if verdict_good:
        print("改善的项: %s" % ", ".join(verdict_good))
    if not verdict_bad and verdict_good:
        print("\n结论: 未发现劣化项, 改动有效.")
    elif not verdict_good:
        print("\n结论: 没有观察到改善, 检查两组数据的工况是否可比 "
              "(看 cmd_vz 打满占比是否接近).")

    # 工况可比性提醒
    a = m1.get("cmd_vz打满±0.30_占比%", 0)
    b = m2.get("cmd_vz打满±0.30_占比%", 0)
    if a > 0 and abs(b - a) / a > 0.5:
        print("\n[注意] 两组的上游指令抖动程度差异较大 (%.1f%% vs %.1f%%),"
              % (a, b))
        print("       对比结论可信度下降, 建议用同样的测试脚本和场地重跑.")


if __name__ == "__main__":
    main()
