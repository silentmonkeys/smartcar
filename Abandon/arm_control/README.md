# arm_control

Reusable serial-servo controller for Rosmaster.

## File Layout (aligned with car_running)

- `running.py`: main wrapper class `run` and global instance `arm`
- `test.py`: simple usage example
- `arm_servo_controller.py`: full-feature base controller implementation

## Quick Start

```python
from running import ActionStep, get_arm

arm = get_arm()

arm.set(1, 90, duration=0.3)
arm.set_all(90, 90, 90, 90, 90, 180, duration=0.5)

wave = [
    ActionStep({2: 70, 3: 130}, 0.3),
    ActionStep({2: 120, 3: 90}, 0.3),
]
arm.add_action("wave", wave)
arm.action("wave", loop=2)

arm.close()
del arm
```

## Servo Angle Limits

- Servo s1-s5: 0 to 180
- Servo s6: 30 to 180
