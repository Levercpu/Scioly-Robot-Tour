#!/usr/bin/env pybricks-micropython
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



# Write your program here.


def turn_180(speed) :
    gyro_sensor.reset_angle(0)
    initial_angle = gyro_sensor.angle()
    while abs(gyro_sensor.angle() - initial_angle) <= 170 :
        left_motor.run(speed)
        right_motor.run(-speed)
    left_motor.stop()
    right_motor.stop()
    if gyro_sensor.angle() > 180 :
        while gyro_sensor.angle() >= 181 :
            left_motor.run(-speed/25)
            right_motor.run(speed/25)
        left_motor.stop()
        right_motor.stop()
    elif gyro_sensor.angle() < 180 :
        while gyro_sensor.angle() <= 179 :
            left_motor.run(speed/20)
            right_motor.run(-speed/20)
        left_motor.stop()
        right_motor.stop()
    wait(0.1)

def turn_left(degrees,speed) :
    gyro_sensor.reset_angle(0)
    initial_angle = gyro_sensor.angle()
    while abs(gyro_sensor.angle() - initial_angle) <=  -(degrees - 15):
        left_motor.run(-speed)
        right_motor.run(speed)
    left_motor.stop()
    right_motor.stop()
    if (-gyro_sensor.angle() > degrees) :
        while gyro_sensor.angle() <= -degrees - 1 :
            left_motor.run(speed/25)
            right_motor.run(-speed/25)
        left_motor.stop()
        right_motor.stop()
    elif (-gyro_sensor.angle() < degrees) :
        while gyro_sensor.angle() >= -degrees + 1 :
            left_motor.run(-speed/25)
            right_motor.run(speed/25)
        left_motor.stop()
        right_motor.stop()
    wait(0.1)

def turn_right(degrees,speed) :
    gyro_sensor.reset_angle(0)
    initial_angle = gyro_sensor.angle()
    while abs(gyro_sensor.angle() - initial_angle) <= degrees - 15:
        left_motor.run(speed)
        right_motor.run(-speed)
    left_motor.stop()
    right_motor.stop()
    if (gyro_sensor.angle() > degrees) :
        while gyro_sensor.angle() <= degrees + 1 :
            left_motor.run(-speed/25)
            right_motor.run(speed/25)
        left_motor.stop()
        right_motor.stop()
    elif (gyro_sensor.angle() < degrees) :
        while gyro_sensor.angle() >= degrees - 1 :
            left_motor.run(2)
            right_motor.run(-2)
        left_motor.stop()
        right_motor.stop()
    wait(0.1)

def drive_forward(distance,maxspeed,minspeed) :
    gyro_sensor.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    speed = (maxspeed-minspeed)*(((left_motor.angle()+right_motor.angle())/2 / (distance * circumference))^9)+minspeed
    while (left_motor.angle() + right_motor.angle())/2 < distance * circumference - 30 :
        left_motor.run(speed+gyro_sensor.angle*20)
        right_motor.run(speed-gyro_sensor.angle*20)
    left_motor.stop()
    right_motor.stop()
    wait(0.1)
def drive_backward(distance,maxspeed,minspeed) :
    gyro_sensor.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    speed = ((maxspeed-minspeed)*(((left_motor.angle()+right_motor.angle())/2 / (distance * circumference))^9)+minspeed)
    while (left_motor.angle() + right_motor.angle())/2 < distance * circumference :
        left_motor.run(-speed-gyro_sensor.angle*20)
        right_motor.run(-speed+gyro_sensor.angle*20)
    left_motor.stop()
    right_motor.stop()
    wait(0.1)


drive_forward(50,50)
drive_backward(50,50)
turn_right(90,50)
turn_left(90,50)
turn_180(50)