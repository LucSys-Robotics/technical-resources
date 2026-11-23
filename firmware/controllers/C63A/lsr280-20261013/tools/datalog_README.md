# LSR280 data-logging tools — development diagnostics

Host-side scripts for the firmware's `LUCSYS_APP/datalog_task.c`. This guide translates the supplied development notes for this source snapshot. Its thresholds, reported observations and diagnostic interpretations are not new measurements or verified specifications for every delivered LSR280.

## Environment

The original setup uses Linux, Python 3.8.10 and `/dev/ttyCH343USB0` (CH343 USB-to-serial).

```bash
pip3 install pyserial pandas matplotlib
```

The plotting script supports Chinese fonts and falls back to English titles when they are unavailable. To install CJK fonts, if needed:

```bash
sudo apt install fonts-noto-cjk
```

Rebuild Matplotlib's font cache if the installed font is not detected. This documentation translation does not change runtime messages, plot strings or CSV field names.

## 1. Preparation

### Firmware configuration

- Check that `DATALOG_ENABLE` in `LUCSYS_APP/Inc/datalog_task.h` is `1`.
- Check that both `HAL_UART_Transmit_DMA(serial,...)` calls in `data_task.c` are commented out. UART4 is used by this diagnostic tool in that configuration.

These are instructions for selecting the documented diagnostic configuration, not changes applied by the English translation.

### Serial permissions

```bash
ls -l /dev/ttyCH343USB0
sudo usermod -aG dialout $USER
```

Log out and back in after changing group membership. Use the actual serial device on your system. The original note also suggests temporary `chmod 666` access; that grants access to every local user, so group-based access is preferred here.

### Check incoming data

```bash
stty -F /dev/ttyCH343USB0 115200 raw -echo
cat /dev/ttyCH343USB0
```

Expect scrolling `D,...` text records. Stop with Ctrl+C before proceeding.

## 2. Capture

A data-group identifier is required:

```bash
python3 datalog_recv.py 01
python3 datalog_recv.py 01 --port /dev/ttyCH343USB0 --outdir ./logs
```

The script prints status every 5 seconds and reports events such as errors, disconnections and emergency-stop indications immediately. Ctrl+C closes the output files cleanly.

| File | Contents |
| --- | --- |
| `01_data.csv` | Periodic 10 Hz data converted to physical units, including derived analysis columns |
| `01_event.csv` | Errors, enable-state transitions, deceleration hold-gate events, driver disconnections and control-source changes |
| `01_stat.csv` | Every 10 seconds: accumulated distance, operating time and current/temperature peaks |
| `01_raw.txt` | Original text for troubleshooting |

### Existing-data protection

If any output file for a group already exists, the script refuses to run and lists the existing files. Choose a different group name. Use `--force` only when intentionally replacing that group's captured data. These runtime messages may remain in Chinese; their behavior is unchanged.

Group names may contain letters, numbers, underscores, periods and hyphens, and must begin with a letter or number. Examples: `01`, `02`, `endurance_a`, `slope-test.2`.

For a long run, first record approximately one minute under a temporary group such as `t00`. Confirm that the measured rate is close to 10 Hz, statistics increase, and dropped-frame and malformed-line counts remain zero. Then use the intended group name.

## 3. Plot

```bash
python3 datalog_plot.py 01 --decimate 20
```

The script finds `logs/01_data.csv` and `logs/01_event.csv`, then writes figures to `charts/01/`. A direct path is also accepted:

```bash
python3 datalog_plot.py logs/01_data.csv
```

An unknown group name causes the available groups under `logs/` to be listed.

Use `--decimate` for long recordings: 8 hours at 10 Hz produces approximately 290,000 points. The original guidance recommends factors of 5 for 1 hour, 20 for 8 hours and 50 for 24 hours.

The following interpretations are diagnostic guidance from the original development note, not conclusive fault diagnoses:

| Figure | What to inspect |
| --- | --- |
| `01_temperature.png` | Whether temperatures settle; large wheel-to-wheel differences can indicate unequal mechanical resistance |
| `02_current.png` | Persistently high current at one wheel: inspect mechanics, tire pressure and encoder configuration. Peaks aligned with pitch may correspond to climbing |
| `03_steer_error.png` | `scl` should decrease with large steering error; otherwise inspect speed-limit activation |
| `04_slip.png` | Slip indicator: peaks aligned with `hold` suggest a slow deceleration hold response; other peaks may indicate an unhandled operating case |
| `05_command_chain.png` | Compare command → smoothed → feedback values to locate limitations in the limiter or motor response |
| `06_power.png` | Compare the observed alarm/disable transitions with this development note's 20.5 V alarm and 20.0 V disable thresholds |
| `07_overview.png` | Velocity, enable-state and error timelines |

The console also reports peak temperature/current, steering error, slip, voltage range, dropped frames and event counts.

## 4. Compare before and after a change

```bash
python3 datalog_compare.py 04 05
```

The script compares groups 04 and 05 and reports ratios and assessments. The original illustrative output is translated below; these numbers are examples, not new test results:

| Metric | Before | After | Ratio | Assessment |
| --- | ---: | ---: | ---: | --- |
| Target jumps greater than 50 degrees: count | 444 | 18 | 0.04x | Significant improvement |
| Mean command transmission rate, per second | 18.30 | 2.10 | 0.11x | Significant improvement |
| Minimum `comm_qua`, % | 82.00 | 97.00 | 1.18x | Improvement |
| HOLD event count | 15.00 | 14.00 | 0.93x | Normal |
| Maximum speed, m/s | 0.73 | 0.71 | 0.97x | Normal |

For metrics expected to decrease, such as jump count, transmission rate or steering error, a reduction exceeding half is classified as significant improvement. Metrics expected to remain stable, including maximum speed and HOLD count, are marked `<<<` for manual review if they change by more than 30%.

The script also compares upstream command variation using the proportion of saturated `cmd_vz` values. A large mismatch makes the comparison unreliable; rerun using the same test sequence and test area.

## 5. Data dictionary

`data.csv` uses physical units and can be opened in Excel. Field names remain unchanged.

### State

| Field | Meaning |
| --- | --- |
| `t_ms` | Firmware timestamp, ms |
| `t_s` | Seconds since the first frame; use this as the plot time axis |
| `seq` | Frame sequence number; gaps indicate missing frames |
| `src` | Control source: 2 = handheld controller, 3 = RC transmitter, 4 = app, 5 = ROS, 6 = CAN, 7 = automatic recharging, 8 = automatic test |
| `en` | Robot enable state; 0 = disabled |
| `err` | Error bitmap, below |
| `hold` | Steering-redirection deceleration hold gate; 1 = waiting for the chassis to slow down. This is a control-logic state, not a claim of a hydraulic valve |
| `park` | Parking mode |
| `scl` | Steering-related speed scale, 0–1; 0 means a stop command is issued in that cycle |
| `mprg` / `mprg_deg` | Maximum steering deviation across four wheels, in counts / degrees |

### Error bitmap: `err`

| Bit | Meaning | Bit | Meaning |
| --- | --- | --- | --- |
| 0 | Low battery | 6 | Driver 2 error |
| 1 | Emergency-stop button pressed | 7 | Driver 3 offline |
| 2 | Software emergency-stop indication | 8 | Driver 3 error |
| 3 | Driver 1 offline | 9 | Driver 4 offline |
| 4 | Driver 1 error | 10 | Driver 4 error |
| 5 | Driver 2 offline | 11 | Steering-homing timeout |

`ERR` records in `event.csv` use readable names such as `LowPower|Drv3_offline`. A software emergency-stop indication is not evidence of physical power isolation.

### Velocity: m/s; angular velocity: rad/s

| Field | Meaning |
| --- | --- |
| `cmd_vx/vy/vz` | Raw command received by the queue |
| `sm_vx/vy/vz` | Values executed after smoothing and speed limiting |
| `fb_vx/vy/vz` | Feedback calculated by forward kinematics |
| `spd_mps` | Resultant linear speed |

### Motors

| Field | Unit | Meaning |
| --- | --- | --- |
| `st_tgt_A~D` | counts | Steering-position target |
| `st_act_A~D` | counts | Actual steering position |
| `st_err_A~D_deg` | degrees | Derived steering tracking error |
| `rpm_tgt_A~D` | rpm | Hub-motor speed target |
| `spd_act_A~D` | m/s | Actual hub linear speed |
| `cur_A~D` | A | Hub current; actual update rate is 5 Hz |
| `tmp_A~D` | degC | Hub temperature; actual update rate is 5 Hz |
| `cur_sum_A` | A | Derived sum of the absolute currents of all four wheels |

The original note uses 16,384 encoder counts per revolution, approximately 45.51 counts per degree. These values apply to the documented reference, not automatically to another encoder or hardware revision.

### Vehicle and communication health

| Field | Meaning |
| --- | --- |
| `vol` | Battery voltage, V |
| `pitch`, `roll`, `yaw` | Attitude angles, degrees |
| `gyro_z` | IMU-measured Z-axis angular velocity, rad/s |
| `slip_radps` | Slip indicator = `fb_vz` − `gyro_z`; a diagnostic quantity for four-wheel drive/four-wheel steering |
| `online_bm` | Driver online bitmap; bits 0–3 represent drivers 1–4; normal value is 15 |
| `err_bm` | Driver error bitmap; normal value is 0 |
| `comm_qua` | Lowest communication-quality value across the four drives, % |
| `fw_drop` | Frames discarded because DMA was busy; should remain 0 |

## 6. Known limitations of this reference

1. Current and temperature update at 5 Hz. TPDO1 has a 200 ms period, so 10 Hz logging repeats each value across two samples. This still supports trend inspection.
2. The original note states that steering motors associated with drivers 1/2 do not provide current or temperature sensing. This is described as a hardware limitation, not a missing software setting. Inspect steady-state `st_err_A~D_deg` when assessing load. The note reports approximately 0.3–2.5° unloaded error; a substantially greater loaded error suggests insufficient steering capability. This historical observation is not a new test or guarantee.
3. The original firmware behavior described here has no audible low-battery warning. Watch `vol` during long runs; the logging tool did not introduce this behavior.
4. During capture, the host must only read UART4, not transmit to it; the board's ROS receive parser is still active on that port.

## 7. Return from the diagnostic configuration

The original note gives these steps:

1. Set `DATALOG_ENABLE` in `datalog_task.h` back to `0`.
2. Uncomment the two `HAL_UART_Transmit_DMA(serial,...)` calls in `data_task.c`, identified by `[DATALOG]` comments.

These steps restore the referenced diagnostic settings; they do not prove that the resulting build matches an individual robot's shipped firmware. No firmware configuration is changed by translating this guide.
