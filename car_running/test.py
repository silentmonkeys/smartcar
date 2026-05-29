from running import run
import time

car = run()

car.ahead()
time.sleep(6)
car.stop()
time.sleep(0.2)

car.left()
time.sleep(5)
car.stop()
time.sleep(0.2)

car.ahead()
time.sleep(5)
car.stop()
time.sleep(0.2)

car.right()
time.sleep(5.5)
car.stop()
time.sleep(0.2)

car.back()
time.sleep(5)
car.stop()
time.sleep(0.2)

car.right()
time.sleep(5)
car.stop()
time.sleep(0.2)

car.ahead()
time.sleep(5)
car.stop()
time.sleep(0.2)

car.left()
time.sleep(5)
car.stop()
time.sleep(0.2)

car.ahead()
time.sleep(6)

car.stop()


car.close()  # 显式释放
del car