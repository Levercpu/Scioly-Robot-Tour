#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

'''

TO-DO LIST

- test on actual track

'''



#objects

ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
gyro_sensor = GyroSensor(Port.S1)

#constants

#cm
wheel_circum = 20.9
square_length = 50
axle_track = 144

#mm/s
absolute_min_speed = 10 #smallest speed the motors will function at
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
        
    while abs(gyro_sensor.angle()) <= abs(degrees) - 20:
        turn_speed_ratio = 0.75 + (degrees - gyro_sensor.angle()) / degrees

        print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

        left_motor.run(-turn_speed_ratio * speed * sign(gyro_sensor.angle() - degrees))
        right_motor.run(turn_speed_ratio * speed * sign(gyro_sensor.angle() - degrees))
    
    align_angle(degrees)
    align_angle(degrees)
    
    wait(100)
    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

def drive(distance, speed):
    gyro_sensor.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    p_or_n = sign(distance)

    while abs((left_motor.angle() + right_motor.angle()) / 2) < abs(distance) * wheel_circum - 50: # constant at the end is used to offset error
        avg_encoder_value = abs((left_motor.angle() + right_motor.angle()) / 2)
        position_ratio = avg_encoder_value / (distance * wheel_circum)
        scaling_factor = 1 - ((2 * position_ratio - p_or_n) ** 6)
        calc_speed = (p_or_n * speed * scaling_factor) + p_or_n * speed / 5

        if (avg_encoder_value < 209):
            print("ACC " + str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
            left_motor.run(calc_speed - gyro_sensor.angle() * 5)
            right_motor.run(calc_speed + gyro_sensor.angle() * 5)
        elif (avg_encoder_value > (abs(distance) - 10) * 20.9):
            print("DEC " + str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
            left_motor.run(calc_speed - gyro_sensor.angle() * 5)
            right_motor.run(calc_speed + gyro_sensor.angle() * 5)
        else:
            print("MAX " + str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
            left_motor.run(p_or_n * 1.1 * speed - gyro_sensor.angle() * 5)
            right_motor.run(p_or_n * 1.1 * speed + gyro_sensor.angle() * 5)

    align_angle(0)
    align_angle(0)

    wait(100)

    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))


def square(drive_distance, drive_speed, turn_angle, turn_speed):
    gyro_sensor.reset_angle(0)

    drive(drive_distance, drive_speed)
    turn(turn_angle, turn_speed)
    drive(drive_distance, drive_speed)
    turn(turn_angle, turn_speed)
    drive(drive_distance, drive_speed)
    turn(turn_angle, turn_speed)
    drive(drive_distance, drive_speed)
    turn(turn_angle, turn_speed)

    print("Angle difference: " + str(gyro_sensor.angle() % 360))

def check_gyro_drift():
    gyro_sensor.reset_angle(0)

    wait(5000)

    print("Gyro Drift per Second: " + str(gyro_sensor.angle() / 5))


# square(50, 500, 91, 250)
turn(90, 250)