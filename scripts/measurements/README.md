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
| - | `--model` | `Auto from ID` | `rs02`, `rs03` or `rs05`; must match the ID, required only for an unregistered ID |
| - | `--torques` | `Required` | Set test torques in N·m |
| - | `--repeats` | `1` | Set repeats per torque |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks |

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
| - | `--model` | `Auto from ID` | `rs02`, `rs03` or `rs05`; must match the ID, required only for an unregistered ID |
| - | `--speeds` | `Required` | Set test speeds in rad/s |
| - | `--repeats` | `1` | Set repeats per speed |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks |

```bash
# Example
python3 scripts/measurements/damping/damping.py --motor-id 4 --model rs03 --speeds 0.15 0.20 0.28
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
| - | `--model` | `Auto from ID` | `rs02`, `rs03` or `rs05`; must match the ID, required only for an unregistered ID |
| - | `--signs` | `1 -1` | Select positive (`1`) or negative (`-1`) motor torque directions |
| - | `--repeats` | `1` | Set repeats per direction |
| - | `--ignore-joint-limit` | Off | Disable joint-limit checks |

```bash
# Example
python3 scripts/measurements/friction/friction.py --motor-id 4 --model rs03
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
| - | `--model` | `Auto from ID` | `rs02`, `rs03` or `rs05`; must match the ID, required only for an unregistered ID |
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
| - | `--model` | `Auto from ID` | `rs02`, `rs03` or `rs05`; must match the ID, required only for an unregistered ID |
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
python3 scripts/measurements/joint/read_joint_values.py --ids left_leg 13
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
