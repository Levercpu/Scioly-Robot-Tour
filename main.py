#!/usr/bin/env pybricks-micropython
from math import copysign, floor
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
gyro_sensor = GyroSensor(Port.S1)
circumference = 20.9
axle_track = 144
turn_speed = 200
min_speed = 500
max_speed = 1250
square_length = 50



# Write your program here.


def signage(x) :
    if (x>0) :
        return 1
    elif (x==0) :
        return 0
    else :
        return -1
def turn_align() :
    speed = turn_speed
    degrees = - gyro_sensor.angle()
    initial_angle = gyro_sensor.angle()
    while abs(gyro_sensor.angle() - initial_angle) <= abs(degrees)-15:
        left_motor.run(-speed * signage((gyro_sensor.angle()-initial_angle)-degrees))
        right_motor.run(speed * signage((gyro_sensor.angle()-initial_angle)-degrees))
    while gyro_sensor.angle()-initial_angle != degrees :
        left_motor.run((-speed/14) * signage((gyro_sensor.angle()-initial_angle)-degrees))
        right_motor.run((speed/14) * signage((gyro_sensor.angle()-initial_angle)-degrees))
    wait(100)
    while gyro_sensor.angle()-initial_angle != degrees :
        left_motor.run((-speed/14) * signage((gyro_sensor.angle()-initial_angle)-degrees))
        right_motor.run((speed/14) * signage((gyro_sensor.angle()-initial_angle)-degrees))
    left_motor.stop()
    right_motor.stop()
    wait(0.1)

def turn(degrees,speed) :
    gyro_sensor.reset_angle(0)
    initial_angle = gyro_sensor.angle()
    while abs(gyro_sensor.angle() - initial_angle) <= abs(degrees)-15:
        left_motor.run(-speed * signage((gyro_sensor.angle()-initial_angle)-degrees))
        right_motor.run(speed * signage((gyro_sensor.angle()-initial_angle)-degrees))
    while gyro_sensor.angle()-initial_angle != degrees :
        left_motor.run((-speed/7) * signage((gyro_sensor.angle()-initial_angle)-degrees))
        right_motor.run((speed/7) * signage((gyro_sensor.angle()-initial_angle)-degrees))
    wait(100)
    while gyro_sensor.angle()-initial_angle != degrees :
        left_motor.run((-speed/14) * signage((gyro_sensor.angle()-initial_angle)-degrees))
        right_motor.run((speed/14) * signage((gyro_sensor.angle()-initial_angle)-degrees))
    left_motor.stop()
    right_motor.stop()
    wait(0.1)

def turn_left() :
    turn(-90,turn_speed)
def turn_right() :
    turn(90,turn_speed)
def turn_180() :
    turn(180,turn_speed)

def drive(distance,maxspeed,minspeed) :
    gyro_sensor.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    # speed = maxspeed
    sign = signage(distance)
    # 2. Calculate progress (0.0 to 1.0)
    avg_angle = (left_motor.angle() + right_motor.angle()) / 2
    current_pos = avg_angle / (distance * circumference)
    # This represents how far through the movement we are
    progress_error = (2 * current_pos) - sign 

    # 3. Calculate speed using a power function for smooth ramping
    speed_range = maxspeed - minspeed
    scaling_factor = 1 - (progress_error ** 6)

    speed = (sign * speed_range * scaling_factor) + sign*minspeed
    while abs((left_motor.angle() + right_motor.angle())/2) < abs(distance) * circumference - 30 :
        progress = (left_motor.angle() + right_motor.angle())/2
        if (progress<209) :
            left_motor.run(speed-gyro_sensor.angle()*5)
            right_motor.run(speed+gyro_sensor.angle()*5)
        elif (progress>(distance-10)*20.9) :
            left_motor.run(speed-gyro_sensor.angle()*5)
            right_motor.run(speed+gyro_sensor.angle()*5)
        else :
            left_motor.run(max_speed-gyro_sensor.angle()*5)
            right_motor.run(max_speed+gyro_sensor.angle()*5)
    left_motor.stop()
    right_motor.stop()
    wait(50)
    turn_align
def drive_forward_raw(distance) :
    drive(distance,max_speed,min_speed)
def drive_backward_raw(distance) :
    drive(-distance,max_speed,min_speed)
def drive_forward(incriments) :
    drive_forward_raw(incriments*50)
def drive_backward(incriments) :
    drive_backward_raw(incriments*50)

drive_backward(1)
turn_left()
drive_backward(1)
turn_left()
drive_backward(1)
turn_left()
drive_backward(1)
turn_left()
while (True):
    print(gyro_sensor.angle())
    wait(1000)
