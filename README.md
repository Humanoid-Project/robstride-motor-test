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

| ID | Joint | Group | Model | Default CAN | edu | pro | max |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `1`–`6` | `left_hip_yaw` … `left_ankle_lower` | `left_leg` | rs02/rs03 | `can0` | ✓ | ✓ | ✓ |
| `7`–`12` | `right_hip_yaw` … `right_ankle_lower` | `right_leg` | rs02/rs03 | `can1` | ✓ | ✓ | ✓ |
| `13` | `neck_pitch` | `head` | rs05 | `can4` | ✓ | ✓ | ✓ |
| `14` | `neck_yaw` | `head` | rs05 | `can4` | - | - | ✓ |
| `15`–`18` | `left_shoulder_pitch`, `left_shoulder_roll`, `left_shoulder_yaw`, `left_elbow` | `left_arm` | rs02 | `can2` | - | ✓ | ✓ |
| `20`–`23` | `right_shoulder_pitch`, `right_shoulder_roll`, `right_shoulder_yaw`, `right_elbow` | `right_arm` | rs02 | `can3` | - | ✓ | ✓ |

Head (±30°) and arm (±45°) joint limits are PLACEHOLDERs until measured. `--ids` also takes `legs`, `arms`, `all` and joint names; the default set is the attached robot's column, read from `~/.config/robonex/robot_model`:

```bash
# Example
mkdir -p ~/.config/robonex
echo ver2_edu > ~/.config/robonex/robot_model
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

<br>

## Scripts
| Folder | Description | README |
| --- | --- | :---: |
| `calibration` | Motor mechanical zero and CAN ID setup | [📖](scripts/calibration/) |
| `measurements` | Read joint values and measure RS02/RS03/RS05 physical parameters | [📖](scripts/measurements/) |
| `motor_control` | Motor drive and hardware integration | [📖](scripts/motor_control/) |
