# robstride-motor-test

## Setup
```bash
# Example
cd ~/humanoid_project
git clone https://github.com/Humanoid-Project/robstride-motor-test.git
git clone https://github.com/Humanoid-Project/imu-n100-test.git IMU_N100_Test
cd robstride-motor-test
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`robonex-common` is pinned in `requirements.txt` — see [`robonex-common/setup/SETUP.md`](https://github.com/Humanoid-Project/robonex-common/blob/main/setup/SETUP.md).

<br>

## Motors

| ID | Joint | Group | Model | Default CAN |
| --- | --- | --- | --- | --- |
| `1`–`6` | `left_hip_yaw` … `left_ankle_lower` | `left_leg` | rs02/rs03 | `can0` |
| `7`–`12` | `right_hip_yaw` … `right_ankle_lower` | `right_leg` | rs02/rs03 | `can1` |
| `13`–`15` | reserved | `waist` | - | - |
| `16`–`20` | `left_shoulder_pitch`, `left_shoulder_roll`, `left_shoulder_yaw`, `left_elbow`, `left_wrist_yaw` | `left_arm` | rs02, rs10p, rs00, rs00, rs05 | `can2` |
| `21`–`25` | `right_shoulder_pitch`, `right_shoulder_roll`, `right_shoulder_yaw`, `right_elbow`, `right_wrist_yaw` | `right_arm` | rs02, rs10p, rs00, rs00, rs05 | `can3` |
| `26`–`27` | `neck_pitch`, `neck_yaw` (`28` reserved) | `head` | rs05 | `can4` |

One robot model, `ver2` (all IDs above); an identity file that still says `ver2_edu` is read as `ver2`. Head pitch ±73°, shoulder pitch (16, 21) ±100°; the other arm and neck-yaw limits (±45° / ±30°) are PLACEHOLDERs until measured. `--ids` also takes `legs`, `arms`, `all` and joint names; the default set is the attached robot's, read from `~/.config/robonex/robot_model`:

```bash
# Example
mkdir -p ~/.config/robonex
echo ver2 > ~/.config/robonex/robot_model
```

<br>

## CAN Interface
```bash
# Example
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000

sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can1 txqueuelen 1000

sudo ip link set can2 up type can bitrate 1000000
sudo ip link set can2 txqueuelen 1000

sudo ip link set can3 up type can bitrate 1000000
sudo ip link set can3 txqueuelen 1000

sudo ip link set can4 up type can bitrate 1000000
sudo ip link set can4 txqueuelen 1000
```

Motor group to channel: `left_leg can0`, `right_leg can1`, `left_arm can2`, `right_arm can3`, `head can4` (`robonex-common` `DEFAULT_BUS_MAP`). Override per machine in `~/.config/robonex/bus_map.json`:

```bash
# Example
mkdir -p ~/.config/robonex
echo '{"head": "can0"}' > ~/.config/robonex/bus_map.json
```

### `can_up.sh`

Brings up every bus-map channel that exists on this machine (1 Mbps, txqueuelen 1000), skips missing ones, and prints each channel's groups and CAN state. Groups: `left_leg`, `right_leg`, `head` and any group named in the bus map file.

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `channel` | Bus map | Bring up only these channels; a missing one is an error |
| - | `re` | Off | Take each selected channel down first, then bring it up again |
| - | `--all` | Off | Also select `left_arm` and `right_arm` |
| - | `--strict` | Off | Exit before any change unless every selected channel exists |
| - | `--dry-run` | Off | Print the commands without running them |

```bash
# Example
./run/can_up.sh
./run/can_up.sh re

# One USB-CAN adapter
./run/can_up.sh can0
./run/can_up.sh re can0

# Assembled robot
./run/can_up.sh --strict
./run/can_up.sh --all --dry-run
```

<br>

## Scripts
| Folder | Description | README |
| --- | --- | :---: |
| `calibration` | Motor mechanical zero and CAN ID setup | [📖](scripts/calibration/) |
| `measurements` | Read joint values and measure RS02/RS03/RS05 physical parameters | [📖](scripts/measurements/) |
| `motor_control` | Motor drive and hardware integration | [📖](scripts/motor_control/) |

<br>

## License

Licensed under the [Apache License 2.0](LICENSE). Copyright 2026 RoboNex. See [NOTICE](NOTICE).
