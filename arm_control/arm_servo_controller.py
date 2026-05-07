import atexit
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Union

from Rosmaster_Lib import Rosmaster


@dataclass
class ActionStep:
    """One servo action step.

    Attributes:
        angles: Target servo angles. Can be a list/tuple of 6 items,
            or a dict mapping servo_id -> angle.
        duration: Optional hold time in seconds after this step is sent.
    """

    angles: Union[Sequence[int], Dict[int, int]]
    duration: float = 0.0


class ArmServoController:
    """High-level serial-servo controller for Rosmaster.

    Features:
    - Safe lifecycle management with close(), context manager, and __del__.
    - Simple single-servo and multi-servo controls.
    - Named action registration and sequence execution.
    """

    # Servo limits from Rosmaster sample notebook.
    _ANGLE_LIMITS = {
        1: (0, 180),
        2: (0, 180),
        3: (0, 180),
        4: (0, 180),
        5: (0, 180),
        6: (30, 180),
    }

    def __init__(self, auto_receive_thread: bool = True, auto_torque_on: bool = True):
        self._bot = Rosmaster()
        self._closed = False
        self._actions: Dict[str, List[ActionStep]] = {}
        self._last_angles: List[int] = [90, 90, 90, 90, 90, 180]

        # Ensure cleanup even if user forgets to call close().
        self._finalizer = atexit.register(self.close)

        if auto_receive_thread:
            self._bot.create_receive_threading()

        if auto_torque_on:
            self.enable_torque(True)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __del__(self):
        # Best-effort cleanup during garbage collection.
        try:
            self.close()
        except Exception:
            pass

    @property
    def is_closed(self) -> bool:
        return self._closed

    def close(self):
        """Idempotent resource cleanup.

        - Stops torque (best effort).
        - Deletes Rosmaster instance.
        - Safe to call multiple times.
        """
        if self._closed:
            return

        try:
            self.enable_torque(False)
        except Exception:
            pass

        try:
            if hasattr(self, "_bot") and self._bot is not None:
                del self._bot
        except Exception:
            pass

        self._closed = True

    def _check_not_closed(self):
        if self._closed:
            raise RuntimeError("ArmServoController is closed.")

    @classmethod
    def _validate_servo_id(cls, servo_id: int):
        if servo_id not in cls._ANGLE_LIMITS:
            raise ValueError(f"servo_id must be 1-6, got: {servo_id}")

    @classmethod
    def _validate_angle(cls, servo_id: int, angle: int):
        low, high = cls._ANGLE_LIMITS[servo_id]
        if not (low <= angle <= high):
            raise ValueError(
                f"servo {servo_id} angle out of range: {angle}, expected [{low}, {high}]"
            )

    def set_servo_angle(self, servo_id: int, angle: int):
        self._check_not_closed()
        self._validate_servo_id(servo_id)
        self._validate_angle(servo_id, angle)
        self._bot.set_uart_servo_angle(servo_id, int(angle))
        self._last_angles[servo_id - 1] = int(angle)

    def get_servo_angle(self, servo_id: int) -> int:
        self._check_not_closed()
        self._validate_servo_id(servo_id)
        return self._bot.get_uart_servo_angle(servo_id)

    def set_all_angles(self, angles: Sequence[int]):
        self._check_not_closed()
        if len(angles) != 6:
            raise ValueError(f"angles must have 6 values, got: {len(angles)}")

        checked = []
        for idx, angle in enumerate(angles, start=1):
            self._validate_angle(idx, int(angle))
            checked.append(int(angle))

        self._bot.set_uart_servo_angle_array(checked)
        self._last_angles = list(checked)

    def get_all_angles(self) -> List[int]:
        self._check_not_closed()
        values = self._bot.get_uart_servo_angle_array()
        if isinstance(values, (list, tuple)) and len(values) == 6:
            checked = [int(v) for v in values]
            # Rosmaster read error can return -1 for one or more servos.
            if all(v >= 0 for v in checked):
                self._last_angles = list(checked)
                return checked
        return list(self._last_angles)

    def enable_torque(self, enabled: bool = True):
        self._check_not_closed()
        self._bot.set_uart_servo_torque(bool(enabled))

    def move_to_pose(
        self,
        angles: Union[Sequence[int], Dict[int, int]],
        duration: float = 0.0,
    ):
        """Move to a target pose.

        Args:
            angles: List/Tuple with 6 angles, or dict like {1: 90, 3: 120}.
            duration: Hold time after command is sent.
        """
        self._check_not_closed()

        if isinstance(angles, dict):
            current = self.get_all_angles()
            target = [int(v) for v in current]
            for sid, angle in angles.items():
                self._validate_servo_id(int(sid))
                self._validate_angle(int(sid), int(angle))
                target[int(sid) - 1] = int(angle)
            self.set_all_angles(target)
        else:
            self.set_all_angles([int(v) for v in angles])

        if duration > 0:
            time.sleep(float(duration))

    def execute_action(self, steps: Iterable[ActionStep], loop: int = 1):
        """Execute a list of action steps.

        Args:
            steps: Iterable of ActionStep.
            loop: Repeat count. Use loop <= 0 for infinite loop.
        """
        self._check_not_closed()
        step_list = list(steps)
        if not step_list:
            return

        if loop <= 0:
            while True:
                for step in step_list:
                    self.move_to_pose(step.angles, step.duration)
        else:
            for _ in range(loop):
                for step in step_list:
                    self.move_to_pose(step.angles, step.duration)

    def register_action(self, name: str, steps: Iterable[ActionStep]):
        self._check_not_closed()
        clean_name = str(name).strip()
        if not clean_name:
            raise ValueError("Action name cannot be empty.")

        step_list = list(steps)
        if not step_list:
            raise ValueError("Action steps cannot be empty.")

        self._actions[clean_name] = step_list

    def run_action(self, name: str, loop: int = 1):
        self._check_not_closed()
        if name not in self._actions:
            raise KeyError(f"Action not found: {name}")
        self.execute_action(self._actions[name], loop=loop)

    def list_actions(self) -> List[str]:
        return sorted(self._actions.keys())


if __name__ == "__main__":
    # Minimal example for quick manual validation.
    with ArmServoController() as arm:
        arm.move_to_pose([90, 90, 90, 90, 90, 180], duration=0.8)
        arm.move_to_pose({1: 60, 2: 120}, duration=0.8)
