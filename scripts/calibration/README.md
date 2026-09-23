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
| `check` | - | `can0 can1` | Check standard motor IDs |
| `find` | `--motor-id` | `0~127` | Find a motor ID |
| `set` | `--current-id` | `Required` | Select the ID to change |
| - | `--new-id` | `Required` | Set the new ID |

```bash
# Example
python3 scripts/calibration/motor_id/motor_id.py check
python3 scripts/calibration/motor_id/motor_id.py find
python3 scripts/calibration/motor_id/motor_id.py find --motor-id 4
python3 scripts/calibration/motor_id/motor_id.py set --current-id 1 --new-id 4
```

<br>

## zero_position

### `zero_position.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--ids` | All 12 | Comma-separated motor IDs to zero |
| - | `--pos-range` | `1` | Set the power-on angle range (`0`: `0..2π`, `1`: `-π..π`) |
| - | `--save` | Off | Persist the zero and angle range to flash |

```bash
# Example
python3 scripts/calibration/zero_position/zero_position.py
python3 scripts/calibration/zero_position/zero_position.py --ids 4,10
python3 scripts/calibration/zero_position/zero_position.py --pos-range 1
```
