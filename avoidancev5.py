import picar_4wd as fc
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.filedb import FileDB
import RPi.GPIO as GPIO
import time, math
import threading
import random


# Init Servo
# print("Init Servo: %s" % ultrasonic_servo_offset)
config = FileDB("config")
ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0)) 
my_servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)

ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0)) 
my_us = Ultrasonic(Pin('D8'), Pin('D9'))

MY_ANGLE_RANGE = 180
MY_STEP = 18
my_us_step = MY_STEP
my_angle_distance = [0,0]
my_current_angle = 0
my_max_angle = MY_ANGLE_RANGE/2
my_min_angle = -MY_ANGLE_RANGE/2
my_scan_list = []
my_errors = []
min_speed = 1
max_speed = 10

class Speed():
    def __init__(self, pin):
        self.speed_counter = 0
        self.speed = 0
        self.last_time = 0
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.timer_flag = True
        self.timer = threading.Thread(target=self.fun_timer, name="Thread1")

    def start(self):
        self.timer.start()
        # print('speed start')

    def print_result(self, s):
        print("Rising: {}; Falling: {}; High Level: {}; Low Level: {}".format(s.count("01"), s.count("10"), s.count("1"), s.count("0")))

    def fun_timer(self):
        while self.timer_flag:
            l = ""
            for _ in range(100):
                l += str(GPIO.input(self.pin))
                time.sleep(0.001)
            # self.print_result(l)
            #count = (l.count("01") + l.count("10")) / 2
            count = l.count("01")
            rps = count / 20.0 * 10
            self.speed = round(2 * math.pi * 3.3 * rps, 2)
            self.speed_counter += count

    def __call__(self):
        return self.speed

    def deinit(self):
        self.timer_flag = False
        self.timer.join()


def my_get_distance_at(angle):
    global my_angle_distance
    my_servo.set_angle(angle)
    time.sleep(0.04)
    distance = my_us.get_distance()
    angle_distance = [angle, distance]
    return distance

def my_get_status_at(angle, ref1=35, ref2=10):
    dist = my_get_distance_at(angle)
    if dist > ref1 or dist == -2:
        return 2
    elif dist > ref2:
        return 1
    else:
        return 0

def my_scan_step(ref_far, ref_near):
    global my_scan_list, my_current_angle, my_us_step
    my_current_angle += my_us_step
    if my_current_angle >= my_max_angle:
        my_current_angle = my_max_angle
        my_us_step = -MY_STEP
    elif my_current_angle <= my_min_angle:
        my_current_angle = my_min_angle
        my_us_step = MY_STEP
    status = my_get_status_at(my_current_angle, ref1=ref_far, ref2=ref_near)#ref1

    my_scan_list.append(status)
    if my_current_angle == my_min_angle or my_current_angle == my_max_angle:
        if my_us_step < 0:
            # print("reverse")
            my_scan_list.reverse()
        # print(scan_list)
        tmp = my_scan_list.copy()
        my_scan_list = []
        return tmp
    else:
        return False

def crash(vect):
    for i in range(len(vect)):
        if vect[i] == 0:
            return True
    return False
    
def score(vect):
    score = 0
    for i in range(len(vect)):
        if vect[i] == 2:
            score += 2
        elif vect[i] == 1:
            score += 1
        elif vect[i] == 0:
            score += -2
    return score

def move_forward_5cm():

    speed3 = Speed(25)
    speed4 = Speed(4) 
    speed3.start()
    speed4.start()
    fc.forward(1)
    time.sleep(0.23)
    #fc.stop()
    print(speed3.speed_counter)
    print(speed4.speed_counter)
    print(" ")
    speed3.deinit()
    speed4.deinit()

    
def move_backwards_10cm():
    speed3 = Speed(25)
    speed4 = Speed(4) 
    speed3.start()
    speed4.start()
    fc.backward(1)
    time.sleep(0.5)
    fc.stop()
    print(speed3.speed_counter)
    print(speed4.speed_counter)
    print(" ")
    speed3.deinit()
    speed4.deinit()
   

def make_left_turn_90degrees():
    speed3 = Speed(25)
    speed4 = Speed(4) 
    speed3.start()
    speed4.start()
    #print('I will make a LEFT turn here')
    fc.turn_left(1)
    time.sleep(0.8)
    fc.stop()
    print(speed3.speed_counter)
    print(speed4.speed_counter)
    print(" ")
    speed3.deinit()
    speed4.deinit()

def make_right_turn_90degrees():
    speed3 = Speed(25)
    speed4 = Speed(4) 
    speed3.start()
    speed4.start()
    #print('I will make a LEFT turn here')
    fc.turn_right(1)
    time.sleep(0.8)
    fc.stop()
    print(speed3.speed_counter)
    print(speed4.speed_counter)
    print(" ")
    speed3.deinit()
    speed4.deinit()
    
def main():

    speed = min_speed
    plan = []
    while True:
        scan_list = my_scan_step(40,25)
        if not scan_list:
            continue
        tmp = scan_list
        if len(tmp) == 10:
            print(tmp[0:10])
            # Check for iminent crash then stop, move backwards and randomly decide to make a right or left turn
            left_score = 0
            right_score = 0
            if crash(tmp[0:5]) and crash(tmp[5:10]):
                print('====> Imminent crash! RETREAT! <=====')
                fc.stop()
                time.sleep(1)
                move_backwards_10cm()
                time.sleep(0.5)
                if random.uniform(0,1) < 0.5:
                    print('I will make a LEFT turn here')
                    make_left_turn_90degrees()

                else:
                    print('I will make a RIGHT turn here')
                    make_right_turn_90degrees()
                      
            else:
                if crash(tmp[0:5]):
                    print('=====> Obstacle ahead! I will make a RIGHT turn here <======')
                    fc.stop()
                    time.sleep(0.5)
                    #move_backwards_10cm()
                    time.sleep(0.5)
                    make_right_turn_90degrees()

                
                if crash(tmp[5:10]):
                    print('======> Obstacle ahead! I will make a LEFT turn here <=======')
                    fc.stop()
                    time.sleep(0.5)
                    #move_backwards_10cm()
                    time.sleep(0.5)                    
                    make_left_turn_90degrees()


                    
                if not(crash(tmp[0:5])) and not(crash(tmp[5:10])):
                    left_score = score(tmp[0:5])
                    right_score = score(tmp[5:10])
                    if left_score == right_score:
                        print('=====> Moving FORWARD! <=====')
                        move_forward_5cm()
                    else:
                        if right_score > left_score:
                            # Make a slight right turn
                            print('=====> Making a RIGHT turn here <=======')
                            fc.stop()
                            time.sleep(0.1)
                            make_right_turn_90degrees()
                            move_forward_5cm()

                        else:
                            # Make a slight left turn
                            print('=====> Making a LEFT turn here <=======')
                            fc.stop()
                            time.sleep(0.1)
                            make_left_turn_90degrees()
                            move_forward_5cm()

if __name__ == "__main__":
    try: 
        main()

    finally: 
        fc.stop()

