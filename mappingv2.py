#!/usr/bin/python

from ast import Global
from email.errors import MissingHeaderBodySeparatorDefect
import picar_4wd as fc
from picar_4wd.pwm import PWM
from picar_4wd.pin import Pin
from picar_4wd.servo import Servo
from picar_4wd.ultrasonic import Ultrasonic
from picar_4wd.filedb import FileDB
import math
import time
import random
import sys
import numpy as np

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
far = 30
near = 20

width = 100
height = 100
threshold = 10
map = np.zeros(width,height)



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

def interpolate (points):
    Global map

    if len(points) <= 1:
        return
    else:
        for i in range(0,len(points)-1):
            p1 = points[i] 
            p2 = points[i+1]
            if math.dist(p1,p2) < threshold:
                delta_y = p2[1] - p1[1]
                delta_x = p2[0] - p1[0]
                if delta_x != 0 and delta_y != 0:
                    delta = delta_x / delta_y
                    if p2[0] > p1[0]:
                        x_zero = p1[0]
                        x_interval = range(p1[0],p2[0]+1)
                    else:
                        x_zero = p2[0]
                        x_interval = range(p1[0],p2[0]+1)
                    if p1[1] < p2[1]:
                        y_zero = p1[1]
                    else:
                        y_zero = p2[1]
                    for x in x_interval:
                        y = y_zero + round(x*delta)
                        map[x,y] = 1

                if delta_x == 0:
                    if p1[1] < p2[1]:
                        y_interval = range(p1[1],p2[1]+1)
                    else:
                        y_interval = range(p1[1],p2[1]+1)
                    for y in y_interval:
                        map[p1[0],y] = 1
                    
                if delta_y == 0:
                    if p1[0] < p2[0]:
                        x_interval = range(p1[0],p2[0]+1)
                    else:
                        x_interval = range(p1[0],p2[0]+1)
                    for x in x_interval:
                        map[x,p1[1]] = 1
        return                    
                    
                        






def map_scan(x,y,yaw,readings):
    Global map
    Global my_us_step
    Global far, near

    surface = []

    for i in range(0,len(readings)):
        if readings[i] < 2:
            angle = i*my_us_step
            if readings[i] == 0:
                radius = near
            else:
                if readings[i] == 1:
                    radius = far
            yo = round(radius * math.sin(math.radians(angle)))
            xo = round(radius * math.cos(math.radians(angle)))

            if yaw <= 0 and yaw <= 90:
                if x + xo > width - 1:
                    x_obst = width - 1
                else:
                    x_obst = x + xo
                if y + yo > height - 1:
                    y_obst = height -1
                else:
                    y_obst = y + yo

            if yaw > 90 and yaw <= 180:
                if x + xo > width - 1:
                    x_obst = width - 1
                else:
                    x_obst = x + xo
                if y - yo < 0:
                    y_obst = 0
                else:
                    y_obst = y - yo
            
            if yaw > 180 and yaw <= 270:
                if x - xo < 0:
                    x_obst = 0
                else:
                    x_obst = x - xo
                if y - yo < 0:
                    y_obst = 0
                else:
                    y_obst = y - yo
                    
            if yaw > 270 and yaw < 360:
                if x - xo < 0 :
                    x_obst = 0
                else:
                    x_obst = x - xo
                if y + yo > height - 1:
                    y_obst = height -1
                else:
                    y_obst = y + yo

            map[x_obst, y_obst] = 1
            surface.append([x_obst, y_obst])
    interpolate(surface)

    return


            


        





    return


def main():

    speed = min_speed
    if len(sys.argv) < 4:
        print("****** Can't continue. Initial position missing ********")
        return
    else:
        xi = int(sys.argv[1])
        yi = int (sys.arg[2])
        yawi = int(sys.arg[3])
        if xi < 0 or yi < 0 or yawi < 0 or yawi > 360:
            print("****** Initial position is incorrect. Please start again! *********")
            return
    xp = xi
    yp = yi
    yawp = yawi

    while True:
        scan_list = my_scan_step(far,near)
        if not scan_list:
            continue
        tmp = scan_list
        if len(tmp) == 10:
            print(tmp)
            map_scan(xp,yp,yawp,tmp)
            # Check for iminent crash then stop, move backwards and randomly decide to make a right or left turn
            left_score = 0
            right_score = 0
            if crash(tmp[0:5]) and crash(tmp[5:10]):
                print('====> Imminent crash! RETREAT! <=====')
                fc.stop()
                time.sleep(1)
                fc.backward(5*speed)
                time.sleep(1.5)
                if random.uniform(0,1) < 0.5:
                    print('I will make a LEFT turn here')
                    fc.turn_left(speed)
                    time.sleep(3)
                    fc.forward(speed)
                
                else:
                    print('I will make a RIGHT turn here')
                    fc.turn_right(speed)
                    time.sleep(3)
                    fc.forward(speed)
                    
            else:
                if crash(tmp[0:5]):
                    print('=====> Obstacle ahead! I will make a RIGHT turn here <======')
                    fc.stop()
                    time.sleep(0.5)
                    fc.backward(5*speed)
                    time.sleep(0.5)
                    fc.turn_right(speed)
                    time.sleep(2)
                    fc.forward(speed)
                
                if crash(tmp[5:10]):
                    print('======> Obstacle ahead! I will make a LEFT turn here <=======')
                    fc.stop()
                    time.sleep(0.5)
                    fc.backward(5*speed)
                    time.sleep(0.5)                    
                    fc.turn_left(speed)
                    time.sleep(2)
                    fc.forward(speed)

                    
                if not(crash(tmp[0:5])) and not(crash(tmp[5:10])):
                    left_score = score(tmp[5:8])
                    right_score = score(tmp[7:10])
                    if left_score == right_score:
                        print('=====> Moving FORWARD! <=====')
                        fc.forward(speed)
                    else:
                        if right_score > left_score:
                            # Make a slight right turn
                            print('=====> Making a SLIGHT RIGHT turn here <=======')
                            fc.turn_right(speed)
                            time.sleep(1)
                            fc.forward(speed)
                        else:
                            # Make a slight left turn
                            print('=====> Making a SLIGHT LEFT turn here <=======')
                            fc.turn_left(speed)
                            time.sleep(1)
                            fc.forward(speed)

if __name__ == "__main__":
    try: 
        main()

    finally: 
        fc.stop()
