# calibration

## Structure

```text
calibration/
├── README.md
├── motor_id/
│   └── motor_id.py
└── zero_position/
    └── zero_position.py
```

<br>

## motor_id

### `motor_id.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| `check` | `--can` | Attached robot | Check that the attached robot's motor IDs answer on one channel or all |
| `find` | `--motor-id` | `0~127` | Find a motor ID |
| - | `--can` | Attached robot | Select the CAN channel to search |
| `set` | `--current-id` | `Required` | Select the ID to change |
| - | `--new-id` | `Required` | Set the new ID (`1~127`) |
| - | `--can` | Attached robot | Select the CAN channel of the motor |

```bash
# Example
python3 scripts/calibration/motor_id/motor_id.py check
python3 scripts/calibration/motor_id/motor_id.py find
python3 scripts/calibration/motor_id/motor_id.py find --motor-id 4
python3 scripts/calibration/motor_id/motor_id.py find --can can2
python3 scripts/calibration/motor_id/motor_id.py set --current-id 1 --new-id 4
python3 scripts/calibration/motor_id/motor_id.py set --can can1 --current-id 1 --new-id 8
python3 scripts/calibration/motor_id/motor_id.py set --can can4 --current-id 1 --new-id 13
python3 scripts/calibration/motor_id/motor_id.py set --can can4 --current-id 1 --new-id 14
python3 scripts/calibration/motor_id/motor_id.py set --can can2 --current-id 1 --new-id 15
python3 scripts/calibration/motor_id/motor_id.py set --can can3 --current-id 1 --new-id 20
```

<br>

## zero_position

### `zero_position.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--ids` | Attached robot | Motors to zero: IDs, groups (`legs`, `head`, `arms`, …) or joint names |
| - | `--pos-range` | `1` | Set the power-on angle range (`0`: `0..2π`, `1`: `-π..π`) |
| - | `--save` | Off | Persist the zero and angle range to flash |

```bash
# Example
python3 scripts/calibration/zero_position/zero_position.py
python3 scripts/calibration/zero_position/zero_position.py --ids 4,10
python3 scripts/calibration/zero_position/zero_position.py --ids head
python3 scripts/calibration/zero_position/zero_position.py --pos-range 1
```
