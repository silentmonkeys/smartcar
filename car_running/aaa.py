from running import run
import time

car = run()

def move_forward():
    car.ahead()
    time.sleep(3)
    car.stop()

def move_left():
    car.left()
    time.sleep(4)
    car.ahead()
    time.sleep(3)
    car.stop()

def move_right():
    car.right()
    time.sleep(4)
    car.ahead()
    time.sleep(2)
    car.stop()