from running import run
import time

car = run()

car.ahead()
time.sleep(5)
car.stop()


car.close()  # 显式释放
del car