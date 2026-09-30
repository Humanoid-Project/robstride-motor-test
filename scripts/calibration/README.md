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
| `check` | `--can` | All mapped | Check standard motor IDs on one channel or all |
| `find` | `--motor-id` | `0~127` | Find a motor ID |
| - | `--can` | All mapped | Select the CAN channel to search |
| `set` | `--current-id` | `Required` | Select the ID to change |
| - | `--new-id` | `Required` | Set the new ID (`1~127`) |
| - | `--can` | All mapped | Select the CAN channel of the motor |

```bash
# Example
python3 scripts/calibration/motor_id/motor_id.py check
python3 scripts/calibration/motor_id/motor_id.py find
python3 scripts/calibration/motor_id/motor_id.py find --motor-id 4
python3 scripts/calibration/motor_id/motor_id.py set --current-id 1 --new-id 4
python3 scripts/calibration/motor_id/motor_id.py set --can can1 --current-id 1 --new-id 8
python3 scripts/calibration/motor_id/motor_id.py set --can can4 --current-id 1 --new-id 13
```

<br>

## zero_position

### `zero_position.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--ids` | All motors | Comma-separated motor IDs to zero |
| - | `--pos-range` | `1` | Set the power-on angle range (`0`: `0..2π`, `1`: `-π..π`) |
| - | `--save` | Off | Persist the zero and angle range to flash |

```bash
# Example
python3 scripts/calibration/zero_position/zero_position.py
python3 scripts/calibration/zero_position/zero_position.py --ids 4,10
python3 scripts/calibration/zero_position/zero_position.py --pos-range 1
```
