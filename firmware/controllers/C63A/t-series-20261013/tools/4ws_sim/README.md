# 四轮四转运动学对拍工程

在 PC 上验证 `LUCSYS_BSP/4ws_v2.c` 的运动学正确性。**直接编译固件里的真实源文件**，不做拷贝或改写 —— 测的就是车上跑的那份代码。

改动运动学后先在这里跑一遍，几秒钟出结果，不用每次都上实车。

## 用法

```bash
cd tools/4ws_sim
make run        # 20 万组，日常自检
make quick      # 1 万组，改代码后快速验证
make full       # 200 万组，提交前完整对拍
```

也可以直接指定样本数和随机种子（种子固定，结果可复现）：

```bash
./4ws_sim 500000 42
```

退出码 0 = 全部通过，1 = 有项目未通过，可直接接进 CI 或提交钩子。

## 五项不变量

| 项 | 检查什么 | 不通过意味着 |
|---|---|---|
| **T1 往返对拍** | 逆解 → 假设轮子到位 → 正解，三轴速度应还原 | 运动学公式本身错了 |
| **T2 角度范围** | AB 轮目标 ∈ [−50°, 220°]，CD 轮 ∈ [−220°, 50°] | 轮子会转出物理限位 |
| **T3 投影一致性** | `WheelSpeedProjection` 与 `InverseKinematics` 算出的轮速必须相同 | **下发的角度与轮速符号不匹配，轮子会瞬间反转** |
| **T4 矢量连续性** | 输入连续变化时，轮速矢量 `speed×(cos a, sin a)` 不应跳变 | 转向目标出现无意义突跳，实车表现为抖动 |
| **T5 零速保持** | 三轴为零时保持上次角度、仅轮速归零 | 停车时轮子会乱摆 |

### T3 和 T4 为什么重要

**T3** 对应 code review 第 7 条。固件的用法是：用 `four_ws_InverseKinematics` 取**转向角**，用 `four_ws_WheelSpeedProjection` 取**轮速**。两者必须对同一组输入给出一致的结果，否则会出现"下发的角度和轮速符号打架，轮子瞬间反转"。这条以前只能靠实车在翻转边界工况上试，现在几秒钟就能验证。

**T4** 对应实车观察到的转向抖动。检查的是**速度矢量**而不是角度本身 —— 因为"目标角翻转 180° + 轮速取反"是合法的等价表示（走近路），矢量不变；只有真正的无意义突跳才会被判出来。这样合法翻转不会误报。

T4 扫描 5 种工况：直行↔自转、直行↔横移、**低速+自转**（实车抖动最剧烈的工况）、倒车↔前进、斜向扫描。

## 判据阈值

在 `main.c` 顶部，需要时调整：

```c
#define EPS_ROUNDTRIP   1e-3f    // 往返误差 m/s, rad/s
#define EPS_PROJECTION  1e-4f    // 投影法与逆解的轮速差
#define ANGLE_MARGIN    0.5f     // 物理范围判定余量(度)
#define EPS_CONTINUITY  0.05f    // 速度矢量连续性 m/s
```

车型参数（轮距/轴距/转向半径）也在 `main.c` 顶部，与 `robot_select_init.h` 的 T280 保持一致。换车型时同步修改。

## 编译说明

- 只依赖 `stdint.h` / `math.h`，无需任何桩代码
- 源文件是 GBK 编码。**不要加 `-finput-charset=GBK`** —— MinGW 8.1 缺 iconv 支持，会报 `failure to convert GBK to UTF-8`。gcc 默认按 UTF-8 读，GBK 注释里的非法字节序列它直接放行，不影响编译
- 已扫描确认工程内不存在"GBK 字符尾字节为 `0x5C` 且位于 `//` 行尾"的续行陷阱（那种情况下 gcc 会把反斜杠当续行符，吞掉下一行代码）

### Windows (MinGW)

本机的 MinGW 在 `E:\ZZZ---engineering software---ZZZ\mingw64`。两点注意：

- make 的可执行文件叫 **`mingw32-make`**，不叫 `make`
- Windows PowerShell 5.1 **不支持 `&&`** 串联命令，要用 `;`

PowerShell：

```powershell
$env:Path = "E:\ZZZ---engineering software---ZZZ\mingw64\bin;$env:Path"
Set-Location "D:\N100_SyncFile\LUCSYS_Project\07_T280\LUCSYS - end\tools\4ws_sim"
mingw32-make run
```

cmd：

```bat
set PATH=E:\ZZZ---engineering software---ZZZ\mingw64\bin;%PATH%
cd /d "D:\N100_SyncFile\LUCSYS_Project\07_T280\LUCSYS - end\tools\4ws_sim"
mingw32-make run
```

或者不用 make，直接一行编译：

```bat
gcc -std=c99 -O2 -Wall -Wextra -I../../LUCSYS_BSP/Inc -o 4ws_sim.exe main.c ../../LUCSYS_BSP/4ws_v2.c -lm
4ws_sim.exe
```

### Linux

```bash
make run
```

## 局限

这个工程只覆盖 `4ws_v2.c` 的**纯运动学**部分。以下内容不在覆盖范围，仍需实车验证：

- `RobotControl_task_v2.c` 里的控制逻辑（起步对准闸、限速器、泄压闸、下发节流）
- 转向电机的实际跟随性能
- CAN 通信与驱动器行为

要观察这些，用 `LUCSYS/tools/` 下的数据观察工具（`datalog_recv.py` / `datalog_plot.py` / `datalog_compare.py`）。
