from arm_servo_controller import ActionStep, ArmServoController


class run(ArmServoController):
    """Arm control wrapper aligned with car_running naming style.

    Use methods inherited from ArmServoController directly, or call
    the convenience aliases in this class for shorter commands.
    """

    def set(self, servo_id: int, angle: int, duration: float = 0.0):
        self.set_servo_angle(servo_id, angle)
        if duration > 0:
            import time

            time.sleep(duration)

    def read(self, servo_id: int) -> int:
        return self.get_servo_angle(servo_id)

    def set_all(self, s1: int, s2: int, s3: int, s4: int, s5: int, s6: int, duration: float = 0.0):
        self.move_to_pose([s1, s2, s3, s4, s5, s6], duration=duration)

    def pose(self, angles, duration: float = 0.0):
        self.move_to_pose(angles, duration=duration)

    def add_action(self, name: str, steps):
        self.register_action(name, steps)

    def action(self, name: str, loop: int = 1):
        self.run_action(name, loop=loop)


# Lazy global instance to avoid opening serial port on import.
arm = None


def get_arm() -> run:
    global arm
    if arm is None or arm.is_closed:
        arm = run()
    return arm


__all__ = ["run", "arm", "get_arm", "ActionStep"]
