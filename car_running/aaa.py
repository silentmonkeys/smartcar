from Rosmaster_Lib import Rosmaster
import time


bot = Rosmaster()

def stop():
    bot.set_car_motion(0.0, 0.0, 0)

def ahead():
    bot.set_car_motion(0.1, 0.0, 0)
    time.sleep(5)
    stop()
    

def main():
    ahead() 
    
if __name__ == "__main__":
    main()
    time.sleep(1)
    stop()