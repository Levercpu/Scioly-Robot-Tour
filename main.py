#!/usr/bin/env pybricks-micropython
from sys import exit
from math import copysign, floor
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


#objects

ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
gyro_sensor = GyroSensor(Port.S1)
#ultrasonic_sensor = UltrasonicSensor(Port.S4)

robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

#constants

#cm
wheel_circum = 20.9
square_length = 50
axle_track = 144

#mm/s
absolute_min_speed = 20 #smallest speed the motors will function at
drive_speed = 500
turn_speed = 250


# functions


def sign(x):
    if (x > 0):
        return 1
    elif (x == 0):
        return 0
    else:
        return -1

def align_angle(target_angle):
    wait(50)

    while gyro_sensor.angle() != target_angle:
        print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
        left_motor.run(absolute_min_speed * sign(target_angle - gyro_sensor.angle()))
        right_motor.run(-absolute_min_speed * sign(target_angle - gyro_sensor.angle()))
        
    left_motor.stop()
    right_motor.stop()

    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

def turn(degrees, speed):
    gyro_sensor.reset_angle(0)
        
    while abs(gyro_sensor.angle()) <= abs(degrees) - 10:
        print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
        left_motor.run(-speed * sign(gyro_sensor.angle() - degrees))
        right_motor.run(speed * sign(gyro_sensor.angle() - degrees))
    
    left_motor.stop()
    right_motor.stop()

    align_angle(degrees)
    align_angle(degrees)
    
    wait(100)
    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

def drive(distance, speed):
    gyro_sensor.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    p_or_n = sign(distance)

    # average of the two encoders for accuracy
    avg_encoder_value = (left_motor.angle() + right_motor.angle()) / 2

    # ratio of current distance covered to final distance covered
    position_ratio = avg_encoder_value / (distance * wheel_circum)

    # use formula for smooth acceleration and deceleration
    scaling_factor = 1 - ((2 * position_ratio - p_or_n) ** 6)
    calc_speed = (p_or_n * 150 * scaling_factor) + p_or_n * speed

    while abs((left_motor.angle() + right_motor.angle()) / 2) < abs(distance) * wheel_circum - 50: # constant at the end is used to offset error
        current_distance = (left_motor.angle() + right_motor.angle()) / 2

        if (current_distance < 209):
            print("ACC " + str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
            left_motor.run(calc_speed - gyro_sensor.angle() * 5)
            right_motor.run(calc_speed + gyro_sensor.angle() * 5)
        elif (current_distance > (distance - 10) * 20.9):
            print("DEC " + str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
            left_motor.run(calc_speed - gyro_sensor.angle() * 5)
            right_motor.run(calc_speed + gyro_sensor.angle() * 5)
        else:
            print("MAX " + str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
            left_motor.run(50 + speed - gyro_sensor.angle() * 5)
            right_motor.run(50 + speed + gyro_sensor.angle() * 5)
   
    left_motor.stop()
    right_motor.stop()

    align_angle(0)
    wait(100)
    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

def square1(speed):
    gyro_sensor.reset_angle(0)

    drive(50, speed)
    turn(90, speed)
    drive(50, speed)
    turn(90, speed)
    drive(50, speed)
    turn(90, speed)
    drive(50, speed)
    turn(90, speed)

    print("Angle difference: " + str(gyro_sensor.angle() % 360))

def square2(speed):
    gyro_sensor.reset_angle(0)

    drive(-50, speed)
    turn(-90, speed)
    drive(-50, speed)
    turn(-90, speed)
    drive(-50, speed)
    turn(-90, speed)
    drive(-50, speed)
    turn(-90, speed)

    print("Angle difference: " + str(gyro_sensor.angle() % 360))

def check_gyro_drift():
    gyro_sensor.reset_angle(0)

    wait(5000)

    print("Gyro Drift per Second: " + str(gyro_sensor.angle() / 5))

#can be used to stop robot if it gets too close to wall
def ultrasonic_check(min_distance):
    if(ultrasonic_sensor.distance() < min_distance or ultrasonic_sensor.distance() == min_distance):
        exit()

