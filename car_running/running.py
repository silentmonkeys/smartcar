from Rosmaster_Lib import Rosmaster


class run:
    """Wrapper around Rosmaster that ensures the underlying instance
    is released when the wrapper is deleted. Use `del car` to clean up.
    """
    def __init__(self):
        self._rosmaster = Rosmaster()

    def ahead(self):
        self._rosmaster.set_car_motion(0.5, 0.0, 0)

    def back(self):
        self._rosmaster.set_car_motion(-0.5, 0.0, 0)

    def left(self):
        self._rosmaster.set_car_motion(0.5, 0.0, 0.2)

    def right(self):
        self._rosmaster.set_car_motion(0.5, 0.0, -0.2)

    def left_rotate(self):
        self._rosmaster.set_car_motion(0.0, 0.0, 0.6) 

    def right_rotate(self):
        self._rosmaster.set_car_motion(0.0, 0.0, -0.6)

    def stop(self):
        self._rosmaster.set_car_motion(0.0, 0.0, 0)

    def close(self):
        """Explicitly release the underlying Rosmaster instance."""
        try:
            del self._rosmaster
        except Exception:
            pass

    def __del__(self):
        # try to stop the car and release resources when wrapper is deleted
        try:
            self.stop()
        except Exception:
            pass
        try:
            del self._rosmaster
        except Exception:
            pass


# 创建全局实例，使用完后可调用 `del car` 或 `car.close()` 来释放
car = run()
