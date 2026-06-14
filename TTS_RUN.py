from TTL import speak_cube, speak_cylinder, speak_sphere
from car_running import run 
import time

car = run()

def move_forward():
    car.ahead()
    time.sleep(3)
    car.right()
    time.sleep(1)
    car.stop()

def move_left():
    car.left()
    time.sleep(4)
    car.ahead()
    time.sleep(2)
    car.stop()

def move_right():
    car.right()
    time.sleep(4)
    car.ahead()
    time.sleep(2)
    car.stop()

def cube_3():
    for _ in range(3):
        speak_cube()
        time.sleep(1.4)

def cylinder_3():
    for _ in range(3):
        speak_cylinder()
        time.sleep(1.4)

def sphere_3():
    for _ in range(3):
        speak_sphere()
        time.sleep(1.4)

