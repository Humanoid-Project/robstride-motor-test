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

## CAN Interface
```bash
# Example
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000

sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can1 txqueuelen 1000

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
| `measurements` | Read joint values and measure RS02/RS03 physical parameters | [📖](scripts/measurements/) |
| `motor_control` | Motor drive and hardware integration | [📖](scripts/motor_control/) |
