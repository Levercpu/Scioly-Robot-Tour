#!/usr/bin/env pybricks-micropython
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
right_motor = Motor(Port.B)
gyro_sensor = GyroSensor(Port.S1)

#constants

#cm
wheel_circum = 20.9
square_length = 50
axle_track = 144

#mm/s
absolute_min_speed = 100 #smallest speed the motors will function at
robot_speed = 500


# functions


def sign(x):
    if (x > 0):
        return 1
    elif (x == 0):
        return 0
    else:
        return -1

def align_angle(target_angle):
    while gyro_sensor.angle()-initial_angle != target_angle:
        left_motor.run(absolute_min_speed * sign(gyro_sensor.angle() - target_angle))
        right_motor.run(absolute_min_speed * sign(gyro_sensor.angle() - target_angle))
        
    left_motor.hold()
    right_motor.hold()

def turn(degrees, speed):
    gyro_sensor.reset_angle(0)
    
    initial_angle = gyro_sensor.angle()
    
    while abs(gyro_sensor.angle() - initial_angle) <= abs(degrees)-15:
        left_motor.run(-speed * sign((gyro_sensor.angle() - initial_angle) - degrees))
        right_motor.run(speed * sign((gyro_sensor.angle() - initial_angle) - degrees))
    
    left_motor.hold()
    right_motor.hold()

    align_angle(degrees)
    align_angle(degrees)
    
    wait(100)


def drive(distance, speed):
    gyro_sensor.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    sign = sign(distance)

    # average of the two encoders for accuracy
    avg_encoder_value = (left_motor.angle() + right_motor.angle()) / 2

    # ratio of current distance covered to final distance covered
    position_ratio = avg_angle / (distance * wheel_circum)

    # use formula for smooth acceleration and deceleration
    scaling_factor = 1 - ((2 * current_pos - sign) ** 6)
    speed = (sign * 2/3 * speed * scaling_factor) + sign * speed # 2/3 * speed = range

    while abs((left_motor.angle() + right_motor.angle()) / 2) < abs(distance) * wheel_circum - 30: # constant at the end is used to offset error
        current_distance = (left_motor.angle() + right_motor.angle()) / 2
        if (current_distance < 209):
            left_motor.run(speed - gyro_sensor.angle() * 5)
            right_motor.run(speed + gyro_sensor.angle()*5)
        elif (current_distance > (distance - 10) * 20.9):
            left_motor.run(speed - gyro_sensor.angle() * 5)
            right_motor.run(speed + gyro_sensor.angle() * 5)
        else:
            left_motor.run(5/3 * speed - gyro_sensor.angle() * 5)
            right_motor.run(5/3 * speed + gyro_sensor.angle() * 5)
   
    left_motor.hold()
    right_motor.hold()

    align_angle()
    wait(100)

def square1(speed):
    gyro_sensor.reset_angle(0)

    initial_angle = gyro_sensor.angle()

    drive(50, robot_speed)
    turn(90, robot_speed)
    drive(50, robot_speed)
    turn(90, robot_speed)
    drive(50, robot_speed)
    turn(90, robot_speed)
    drive(50, robot_speed)
    turn(90, robot_speed)

    print("Angle difference: " String((gyro_sensor.angle() - initial_angle) % 360))

def square2(speed):
    gyro_sensor.reset_angle(0)

    initial_angle = gyro_sensor.angle()

    drive(-50, robot_speed)
    turn(-90, robot_speed)
    drive(-50, robot_speed)
    turn(-90, robot_speed)
    drive(-50, robot_speed)
    turn(-90, robot_speed)
    drive(-50, robot_speed)
    turn(-90, robot_speed)

    print("Angle difference: " String((gyro_sensor.angle() - initial_angle) % 360))


def check_gyro_drift():
    initial_angle = gyro_sensor.angle()

    wait(5000)

    print("Gyro Drift per Second: " + String((gyro_sensor.angle() - initial_angle) / 5))