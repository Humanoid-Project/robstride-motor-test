# motor_control

## Structure

```text
motor_control/
├── README.md
├── motor_run_gui.py
└── set_motor_pose.py
```

<br>

## motor_control

### `set_motor_pose.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--ids` | `1`–`12` | Motor IDs to move: leg motors go to the policy default pose, other registered motors (`13` head) to 0 rad |

```bash
# Example
python3 scripts/motor_control/set_motor_pose.py
python3 scripts/motor_control/set_motor_pose.py --ids 13
```

<br>

### `motor_run_gui.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | `5` | Select one motor or two motors on the same CAN channel |

```bash
# Example
python3 scripts/motor_control/motor_run_gui.py --motor-id 4
python3 scripts/motor_control/motor_run_gui.py --motor-id 5 6
```
