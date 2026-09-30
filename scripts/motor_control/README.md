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
| - | `--ids` | `legs` | Motors to move: IDs, groups or joint names; leg motors go to the policy default pose, head and arm motors to 0 rad |
| - | `--allow-placeholder-limits` | Off | Allow head (`13`, `14`) and arm (`15`–`18`, `20`–`23`) motors, whose limits are PLACEHOLDERs |

```bash
# Example
python3 scripts/motor_control/set_motor_pose.py
python3 scripts/motor_control/set_motor_pose.py --ids left_leg
python3 scripts/motor_control/set_motor_pose.py --ids head --allow-placeholder-limits
```

<br>

### `motor_run_gui.py`

| Command | Option | Default | Description |
| --- | --- | --- | --- |
| - | `--motor-id` | `5` | Select one motor or two motors on the same CAN channel (any registered ID `1`–`18`, `20`–`23`) |

```bash
# Example
python3 scripts/motor_control/motor_run_gui.py --motor-id 4
python3 scripts/motor_control/motor_run_gui.py --motor-id 5 6
python3 scripts/motor_control/motor_run_gui.py --motor-id 15
```
