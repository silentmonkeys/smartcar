from running import run
import time

car = run()

# #离开出发区域
# car.ahead()
# time.sleep(6)
# car.stop()
# time.sleep(0.2)
# #s1
# car.left()
# time.sleep(5)
# car.stop()
# time.sleep(0.2)
# #s2
# car.ahead()
# time.sleep(5)
# car.stop()
# time.sleep(0.2)
# #s3
car.right()
time.sleep(5)
car.stop()
time.sleep(0.2)
# #s4
# car.back()
# time.sleep(5)
# car.stop()
# time.sleep(0.2)
# #s5
# car.right()
# time.sleep(5)
# car.stop()
# time.sleep(0.2)
# #s6
# car.ahead()
# time.sleep(5)
# car.stop()
# time.sleep(0.2)
# #s7
# car.left()
# time.sleep(5)
# car.stop()
# time.sleep(0.2)
# #s8
# car.ahead()
# time.sleep(6)
# car.stop()


car.close()  # 显式释放
del car