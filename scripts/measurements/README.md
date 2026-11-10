# measurements

## Structure

```text
measurements/
├── README.md
├── common.py
├── armature/
│   ├── armature.py
│   └── analyze_armature.py
├── check/
│   └── shutdown.py
├── damping/
│   ├── damping.py
│   └── analyze_damping.py
├── friction/
│   ├── friction.py
│   └── analyze_friction.py
├── kp/
│   ├── kp.py
│   └── analyze_kp.py
├── kd/
│   ├── kd.py
│   └── analyze_kd.py
├── latency/
│   ├── latency.py
│   ├── analyze_latency.py
│   ├── bus_timing.py
│   └── analyze_bus_timing.py
├── torque/
│   └── torque.py
├── joint/
│   ├── read_joint_values.py
│   └── scan_joint_limits.py
└── noise/
    ├── motor/
    │   ├── motor_noise.py
    │   ├── analyze_can_noise.py
    │   └── analyze_can_rate.py
    └── imu/
        ├── CMakeLists.txt
        ├── imu_noise.py
        ├── n100_binding.cpp
        └── analyze_imu_noise.py
```

<br>

## armature

Armature is the rotational inertia that resists angular acceleration, measured in kg·m².

### `armature.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | `Required` | Select the motor |
| - | `--model` | `Auto from ID` | `rs00`, `rs02`, `rs03`, `rs05` or `rs10p`; must match the ID, required only for an unregistered ID |
| - | `--torques` | `Required` | Set test torques in N·m |
| - | `--repeats` | `1` | Set repeats per torque |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks |
| - | `--max-vel` | `5.0` | Stop a run at this output speed in rad/s, at most 10 |
| - | `--can-timeout-ms` | `1000` | Motor CAN watchdog for the run (0x7028, not saved, cleared after the run); `0` turns it off |

```bash
# Example
python3 scripts/measurements/armature/armature.py --motor-id 11 --model rs02 --torques 0.3 0.5 1.0
```

<br>

### `analyze_armature.py`

```bash
# Example
python3 scripts/measurements/armature/analyze_armature.py "scripts/measurements/armature/data/*.csv"
```

<br>

## check

### `shutdown.py`

Applies velocity damping, disables the selected motors, and verifies their operating mode before power-off. Prints a per-channel table with one status per motor (`stopped`, `active`, `unconfirmed`, `send_failed`, `channel_unavailable`, `interrupted`); exits `1` unless every selected motor is `stopped`. Ctrl-C still sends a stop to every remaining motor on an open bus.

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--ids` | Attached robot | Select motors to disable: IDs, groups (`legs`, `head`, `arms`, …) or joint names |
| - | `--brake-time` | `0.3` | Set braking time in seconds |
| - | `--kd` | `3.0` | Set braking damping gain |

```bash
# Example
python3 scripts/measurements/check/shutdown.py
python3 scripts/measurements/check/shutdown.py --ids head arms
```

<br>

## damping

Damping is the velocity-proportional resisting torque, measured in N·m/(rad/s).

### `damping.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | `Required` | Select the motor |
| - | `--model` | `Auto from ID` | `rs00`, `rs02`, `rs03`, `rs05` or `rs10p`; must match the ID, required only for an unregistered ID |
| - | `--speeds` | `Required` | Set test speeds in rad/s |
| - | `--repeats` | `1` | Set repeats per speed |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks |
| - | `--max-turns` | `0.5` | Stop a run after this many output turns from its start, at most 1.5 |
| - | `--hold-time` | `1.5` | Recording time at constant speed in s, at most 5 |
| - | `--can-timeout-ms` | `1000` | Motor CAN watchdog for the run (0x7028, not saved, cleared after the run); `0` turns it off |

```bash
# Example
python3 scripts/measurements/damping/damping.py --motor-id 4 --model rs03 --speeds 0.15 0.20 0.28

# Unloaded motor on the bench
python3 scripts/measurements/damping/damping.py --motor-id 5 --ignore-joint-limit --max-turns 1.2 --speeds 0.5 -0.5 1 -1 2 -2 3 -3 --repeats 2
```

<br>

### `analyze_damping.py`

```bash
# Example
python3 scripts/measurements/damping/analyze_damping.py "scripts/measurements/damping/data/*.csv"
```

<br>

## friction

Friction is the breakaway torque required to start a stationary joint moving, measured in N·m.

### `friction.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | `Required` | Select the motor |
| - | `--model` | `Auto from ID` | `rs00`, `rs02`, `rs03`, `rs05` or `rs10p`; must match the ID, required only for an unregistered ID |
| - | `--signs` | `1 -1` | Select positive (`1`) or negative (`-1`) motor torque directions |
| - | `--repeats` | `1` | Set repeats per direction |
| - | `--max-torque` | `1.0` | Torque search upper bound in N·m; the time limit grows with it |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks |
| - | `--pose` | `current` | Start pose before each search: `current`, `zero`, or `stand` (policy default pose); refused outside the joint limits or the 12° foot-roll clip |
| - | `--hold-ids` | - | Motor IDs on the same channel held at the start pose with their deploy gains during the search |
| - | `--can-timeout-ms` | `1000` | Motor CAN watchdog for the run (0x7028, not saved, cleared after the run); `0` turns it off |

```bash
# Example
python3 scripts/measurements/friction/friction.py --motor-id 4 --model rs03

python3 scripts/measurements/friction/friction.py --motor-id 11 --repeats 3 --max-torque 3

# Ankle at the standing pose, partner motor held
python3 scripts/measurements/friction/friction.py --motor-id 5 --pose stand --hold-ids 6 --repeats 3

python3 scripts/measurements/friction/friction.py --motor-id 11 --pose stand --hold-ids 12 --repeats 3 --max-torque 3
```

<br>

### `analyze_friction.py`

```bash
# Example
python3 scripts/measurements/friction/analyze_friction.py "scripts/measurements/friction/data/*.csv"
```

<br>

## kp

kp is the position gain the joint actually honours, as a fraction of the value commanded in MIT mode.

### `kp.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | `Required` | Select the motor |
| - | `--model` | `Auto from ID` | `rs00`, `rs02`, `rs03`, `rs05` or `rs10p`; must match the ID, required only for an unregistered ID |
| - | `--kp` | `Required` | Set the position gain to command |
| - | `--offsets-deg` | `-6 -4 -2 2 4 6` | Set step offsets from the reference, in degrees |
| - | `--repeats` | `2` | Set repeats per offset |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks |
| - | `--ramp-rate` | `0.15` | Set the commanded position slew between offsets, rad/s |

```bash
# Example
python3 scripts/measurements/kp/kp.py --motor-id 4 --model rs03 --kp 100
```

<br>

### `analyze_kp.py`

```bash
# Example
python3 scripts/measurements/kp/analyze_kp.py "scripts/measurements/kp/data/*.csv"
```

<br>

## kd

kd is the velocity gain the joint actually honours. The position target is held and only the
velocity target is stepped, so the joint shifts by kd * vel_target / kp without running.

### `kd.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | `Required` | Select the motor |
| - | `--model` | `Auto from ID` | `rs00`, `rs02`, `rs03`, `rs05` or `rs10p`; must match the ID, required only for an unregistered ID |
| - | `--kp` | `Required` | Set the position gain to hold with |
| - | `--kd` | `Required` | Set the velocity gain to command |
| - | `--vel-targets` | `-1.0 -0.5 0.5 1.0` | Set velocity targets in rad/s |
| - | `--repeats` | `2` | Set repeats per target |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks |
| - | `--ramp-time` | `0.5` | Set seconds to ramp the velocity target in; near `0.02` measures latency |
| - | `--sample-all` | Off | Record the whole hold instead of the settled window |

```bash
# Example
python3 scripts/measurements/kd/kd.py --motor-id 4 --model rs03 --kp 100 --kd 2.0

# Latency variant
python3 scripts/measurements/kd/kd.py --motor-id 4 --model rs03 --kp 100 --kd 2.0 \
  --ramp-time 0.02 --sample-all
```

<br>

### `analyze_kd.py`

```bash
# Example
python3 scripts/measurements/kd/analyze_kd.py "scripts/measurements/kd/data/*.csv"
```

<br>

## latency

Latency is the time from a command frame leaving the CAN adapter to the motor's reported torque changing. The position is held and only the feed-forward torque (`--step-kind torque`), the velocity target (`--step-kind velocity`, the kd path) or the position target (`--step-kind position`, the kp path; the joint moves by the step and back) is stepped, so the joint shifts by torque / kp, kd·v / kp or the step without running; every command and reply carries its kernel CAN timestamp.

### `latency.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | `Required` | Select the motor |
| - | `--model` | `Auto from ID` | `rs00`, `rs02`, `rs03`, `rs05` or `rs10p`; must match the ID, required only for an unregistered ID |
| - | `--kp` | `Required` | Set the position gain to hold with |
| - | `--kd` | `Required` | Set the velocity gain to hold with |
| - | `--step-kind` | `torque` | `torque` steps the feed-forward torque, `velocity` steps the velocity target (torque kd·v), `position` steps the position target (torque kp·step) |
| - | `--torque-steps` | `0.5 -0.5 1.0 -1.0` | Set signed feed-forward torque steps in N·m, each at most 0.25 × rated torque (RS02 1.5, RS03 3.25); `torque` only |
| - | `--vel-steps` | `0.25 -0.25 0.5 -0.5` | Set signed velocity-target steps in rad/s, kd × \|v\| at most 0.25 × rated torque, within the frame velocity range; `velocity` only |
| - | `--pos-steps` | `1 -1 2 -2` | Set signed position-target steps in degrees, each at most 5° and kp × \|step\| at most 0.25 × rated torque (RS03 at kp 100: 1.86°, at kp 150: 1.24°; RS02 at kp 40: 2.15°); joint limits checked with 2 × step; aborts beyond 2 × step + 1° or above the expected peak speed; `position` only |
| - | `--repeats` | `8` | Set repeats per step, at most 50 |
| - | `--hold-time` | `0.3` | Set seconds each step is held, at most 2 |
| - | `--rest-time` | `0.3` | Set seconds at zero step before and after each step, 0.1 to 2 |
| - | `--phase-steps` | `4` | Number of gaps inserted after a step frame, cycled over the repeats to shift the sampling instants |
| - | `--phase-span-ms` | `0.6` | Spread the gaps evenly over this many milliseconds |
| - | `--hw-timestamps` | Off | Also record the adapter's own timestamps (needs gs_usb hardware timestamp support) |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks |
| - | `--can-timeout-ms` | `1000` | Motor CAN watchdog for the run (0x7028, not saved, cleared after the run); `0` turns it off |

```bash
# Example
python3 scripts/measurements/latency/latency.py --motor-id 4 --model rs03 --kp 100 --kd 2.0

# Velocity-target step through kd
python3 scripts/measurements/latency/latency.py --motor-id 9 --kp 100 --kd 2.0 --step-kind velocity --vel-steps 0.5 -0.5 1.0 -1.0

# Position-target step through kp
python3 scripts/measurements/latency/latency.py --motor-id 9 --kp 100 --kd 2.0 --step-kind position --pos-steps 1 -1 1.8 -1.8

# Unloaded motor on the bench
python3 scripts/measurements/latency/latency.py --motor-id 5 --kp 40 --kd 2.0 --ignore-joint-limit
```

| Output | Description |
| --- | --- |
| `data/latency_id<ID>_<model>_<time>.csv` | One row per command/reply cycle: `tx_ts`, `rx_ts` (kernel timestamps, s), `tx_hw_ts`, `rx_hw_ts` (adapter, with `--hw-timestamps`), `t_send_mono`, `t_rx_mono` (host), command fields (`cmd_vel` carries the velocity step), `step_nm` = the step in N·m, rad/s or degrees (`# step_kind velocity` or `# step_kind position` header row), `cmd_pos` carries the position target, feedback position, velocity, torque, temperature, mode, fault, `status` (`ok`, `no_reply`, `no_echo`) |

<br>

### `analyze_latency.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--clock` | `auto` | `sw` kernel timestamps, `hw` adapter timestamps, `auto` uses `hw` when recorded |
| - | `--window-ms` | `50` | Search this long after each step for the response |
| - | `--bin-ms` | `0.15` | Bin width of the pooled response curve |
| - | `--coulomb` | `0` | Coulomb friction in N·m for the position-mode model fit |
| - | `--stiction` | `0` | Breakaway torque in N·m for the position-mode model fit |
| - | `--fit-ms` | `150` | Fit window after the step frame in ms (position mode) |
| - | `--j-candidates` | RS02 `0.0042 0.0173`, RS03 `0.02 0.064` | Inertias in kg·m² scored by the position-mode fit |
| - | `--no-free-j` | Off | Score only `--j-candidates`, no free inertia fit |

```bash
# Example
python3 scripts/measurements/latency/analyze_latency.py "scripts/measurements/latency/data/*.csv"

# Position step with the unit's friction in the model
python3 scripts/measurements/latency/analyze_latency.py scripts/measurements/latency/data/latency_id9_rs03_<time>.csv --coulomb 0.3 --stiction 0.56
```

| Output | Description |
| --- | --- |
| round trip | Command TX to its own reply RX: median, p95, max |
| onset | Step-frame TX to the first reply whose torque passed the threshold (lower bound = the reply before it); pooled over repeats |
| minus RT | Pooled onset minus the median round trip: delay counted from the moment the motor answered the step frame |
| t10, t50, t90, rise | Crossing times of the pooled response curve and the 10–90 % rise time |
| step | `torque`: N·m; `velocity`: rad/s with the expected torque kd·v; `position`: degrees with the initial torque kp·step; the expected torque sets the threshold, peak and plateau |
| peak, plateau | Largest and 5–20 ms median response as a fraction of the step (kd·v in `velocity`) |
| vel-onset, pos-onset | First reply whose velocity or position left its pre-step noise band |
| position response | `position` only, per step and edge, medians over repeats as fractions of the commanded step: t10, t50, t90, rise, peak time, overshoot %, settling into ±5 %, steady error % |
| model fit | `position` only, per step and edge and pooled over all edges of one step size: ideal PD + inertia + Coulomb/stiction with the group's torque onset as delay; best inertia J and position rms of each candidate (mdeg); ideal linear PD peak time and overshoot of each candidate |

<br>

### `bus_timing.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--ids` | `left_leg` | Select motors on one CAN channel: IDs, groups or joint names |
| - | `--mode` | `passive` | `passive`: motors stay disabled, frames carry kp 0, kd 0, torque 0 (refused if a motor reports run mode); `hold`: enable and hold the current position with `--kp`/`--kd` |
| - | `--pattern` | `both` | `burst` sends to every motor back to back, then collects; `sequential` waits for each reply; `both` alternates cycle by cycle |
| - | `--period-ms` | `20` | Cycle period in ms (deploy 50 Hz); `0` runs cycles back to back |
| - | `--duration` | `10` | Run time in s, at most 120 |
| - | `--reply-timeout-ms` | `10` | Wait for replies after the last frame of a burst or after each frame, 1 to 50 |
| - | `--kp` | - | Position gain; `hold` only, required there |
| - | `--kd` | - | Velocity gain; `hold` only, required there |
| - | `--hw-timestamps` | Off | Also record the adapter's own timestamps (needs gs_usb hardware timestamp support) |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks; `hold` only |
| - | `--out` | `data/bus_timing_<channel>_<mode>_<time>.csv` | CSV path; an existing file is never overwritten |
| - | `--can-timeout-ms` | `1000` | Motor CAN watchdog for the run (0x7028, not saved, cleared after the run); `hold` only, `0` turns it off |

```bash
# Example
python3 scripts/measurements/latency/bus_timing.py

# Right leg, deploy rate, burst only
python3 scripts/measurements/latency/bus_timing.py --ids right_leg --pattern burst

# Line saturation
python3 scripts/measurements/latency/bus_timing.py --ids left_leg --period-ms 0 --duration 5

# Under holding torque
python3 scripts/measurements/latency/bus_timing.py --ids left_leg --mode hold --kp 40 --kd 2.0 --duration 10
```

| Output | Description |
| --- | --- |
| `data/bus_timing_<channel>_<mode>_<time>.csv` | One row per command/reply exchange: `cycle`, `pattern`, `slot` (position in the cycle), `motor_id`, `status` (`ok`, `no_reply`, `no_echo`), `tx_ts`, `rx_ts` (kernel timestamps, s), `tx_hw_ts`, `rx_hw_ts` (adapter, with `--hw-timestamps`), `t_cycle_mono`, `t_send_mono`, `t_rx_mono` (host), `tx_id`, `tx_data`, `rx_id`, `rx_data` (raw frames), command and feedback fields, mode, fault |

<br>

### `analyze_bus_timing.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--clock` | `auto` | `sw` kernel timestamps, `hw` adapter timestamps, `auto` uses `hw` when recorded |
| - | `--bitrate` | `1000000` | CAN bitrate for frame times and bus load |

```bash
# Example
python3 scripts/measurements/latency/analyze_bus_timing.py "scripts/measurements/latency/data/bus_timing_*.csv"
```

| Output | Description |
| --- | --- |
| round trip | Per motor and pattern: command TX echo to its own reply, median, p95, max; missed replies |
| rt-wire | Round trip minus both frame times (exact bits with stuffing): the motor's turnaround in `sequential`, turnaround plus queueing in `burst` |
| burst slots | Per slot: TX echo and reply time from the cycle's first TX echo |
| cycle | First TX echo to last reply of a complete cycle, host-clock cycle, share of the cycle the wire is busy |
| bus load | Frame bits over the capture time as a share of the line |
| burst vs sequential | Cycle medians and per-motor round trips side by side |

<br>

## torque

`torque.py` passively displays type `0x02` torque feedback from every motor of the attached robot. It sends no CAN frames, so a motor controller such as `mujoco_to_real.py` must run separately. The displayed value is the motor controller's internal torque estimate, not an independent load-cell measurement.

### `torque.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--refresh` | `10` | Set the terminal refresh rate in Hz |
| - | `--stale-after` | `0.3` | Set the stale-feedback threshold in seconds |

```bash
# Example
python3 scripts/measurements/torque/torque.py
```

<br>

## joint

### `read_joint_values.py`

Reads the type `0x11` mechanical position of each motor, grouped by CAN channel; it never enables a motor.

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--ids` | Attached robot | Select motors: IDs, groups (`left_leg`, `right_leg`, `left_arm`, `right_arm`, `head`, `legs`, `arms`, `all`) or joint names |
| - | `--can` | All | Read only the selected motors on this CAN channel |
| - | `--watch` | Off | Continuously refresh joint values |
| - | `--hz` | `10` | Set the `--watch` screen refresh rate |
| - | `--duration` | `0` | Stop `--watch` after this many seconds (`0`: until Ctrl-C) |
| - | `--csv` | - | Write every reading as a timestamped row |

```bash
# Example
python3 scripts/measurements/joint/read_joint_values.py
python3 scripts/measurements/joint/read_joint_values.py --ids head
python3 scripts/measurements/joint/read_joint_values.py --ids left_leg 26
python3 scripts/measurements/joint/read_joint_values.py --ids left_knee_pitch,right_knee_pitch
python3 scripts/measurements/joint/read_joint_values.py --can can1
python3 scripts/measurements/joint/read_joint_values.py --watch
python3 scripts/measurements/joint/read_joint_values.py --watch --hz 20 --ids arms

# Log for 30 s
python3 scripts/measurements/joint/read_joint_values.py \
    --watch \
    --duration 30 \
    --csv joint_log.csv
```

| Output | Description |
| --- | --- |
| `--csv` file | `time_s`, `unix_time`, `channel`, `motor_id`, `joint`, `position_rad` (empty: no response) |

<br>

### `scan_joint_limits.py`

Tracks the minimum and maximum mechanical positions while each joint is moved by hand, then saves the results as CSV.

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id`, `--motor-ids` | All motors | Select motors to scan |

```bash
# Example
python3 scripts/measurements/joint/scan_joint_limits.py --motor-id 5 6
```

<br>

## noise/motor

### `motor_noise.py`

Enables stationary motors with velocity damping and records type `0x02` position, velocity, torque, temperature, and timestamps to CSV.

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | All motors | Select one or more motors to capture |
| - | `--duration` | `60` | Set simultaneous capture time per active CAN channel |

```bash
# Example
python3 scripts/measurements/noise/motor/motor_noise.py --motor-id 1 2 3 --duration 60
```

<br>

### `analyze_can_noise.py`

Calculates per-motor position mean, position noise, peak-to-peak variation, and velocity noise from motor noise files.

```bash
# Example
python3 scripts/measurements/noise/motor/analyze_can_noise.py "scripts/measurements/noise/motor/data/*.csv"
```

<br>

### `analyze_can_rate.py`

Calculates successful response rate, missed replies, update frequency, and timing jitter for each motor and CAN channel.

```bash
# Example
python3 scripts/measurements/noise/motor/analyze_can_rate.py "scripts/measurements/noise/motor/data/*.csv"
```

<br>

## noise/imu

### Python Binding

```bash
# Example
cmake -S scripts/measurements/noise/imu \
  -B scripts/measurements/noise/imu/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython3_EXECUTABLE="$(pwd)/.venv/bin/python"
cmake --build scripts/measurements/noise/imu/build -j
```

<br>

### `imu_noise.py`

Records stationary N100 raw and fused gyroscope data, acceleration, temperature, and timestamps to CSV.

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--port` | `/dev/ttyUSB0` | Select the IMU serial port |
| - | `--duration` | `60` | Set capture time in seconds |

```bash
# Example
python3 scripts/measurements/noise/imu/imu_noise.py --duration 60
```

<br>

### `analyze_imu_noise.py`

Combines IMU noise files and calculates per-axis gyroscope bias and noise for raw and fused signals.

```bash
# Example
python3 scripts/measurements/noise/imu/analyze_imu_noise.py "scripts/measurements/noise/imu/data/*.csv"
```
