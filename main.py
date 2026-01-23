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



# Write your program here.




def turn(degrees,speed) :
    gyro_sensor.reset_angle(0)
    initial_angle = gyro_sensor.angle()
    while abs(gyro_sensor.angle() - initial_angle) <= abs(degrees)-15:
        left_motor.run(-speed * copysign(1,(gyro_sensor.angle()-initial_angle)-degrees))
        right_motor.run(speed * copysign(1,(gyro_sensor.angle()-initial_angle)-degrees))
    while gyro_sensor.angle()-initial_angle != degrees :
        left_motor.run((-speed/7) * copysign(1,(gyro_sensor.angle()-initial_angle)-degrees))
        right_motor.run((speed/7) * copysign(1,(gyro_sensor.angle()-initial_angle)-degrees))
    wait(100)
    while gyro_sensor.angle()-initial_angle != degrees :
        left_motor.run((-speed/7) * copysign(1,(gyro_sensor.angle()-initial_angle)-degrees))
        right_motor.run((speed/7) * copysign(1,(gyro_sensor.angle()-initial_angle)-degrees))
    left_motor.stop()
    right_motor.stop()
    wait(0.1)

def turn_left() :
    turn(-90,turn_speed)
def turn_right() :
    turn(90,turn_speed)

def turn_180() :
    turn(180,turn_speed)

def drive_forward(distance,maxspeed,minspeed) :
    gyro_sensor.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    # speed = maxspeed
    speed = floor((maxspeed-minspeed)*(((left_motor.angle()+right_motor.angle())/2 / (distance * circumference))**9))+minspeed
    while abs((left_motor.angle() + right_motor.angle())/2) < abs(distance) * circumference - 30 :
        left_motor.run(speed-gyro_sensor.angle()*20)
        right_motor.run(speed+gyro_sensor.angle()*20)
    left_motor.stop()
    right_motor.stop()
    wait(0.1)
def drive_backward(distance,maxspeed,minspeed) :
    drive_forward(-distance,-maxspeed,-minspeed)


drive_backward(50,400,200)
while (True):
    print(gyro_sensor.angle())
    wait(1000)
