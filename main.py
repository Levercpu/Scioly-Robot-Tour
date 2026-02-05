#!/usr/bin/env pybricks-micropython
from math import sin, cos, pi
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
DriveBase
#constants

#cm
wheel_circum = 20.9
square_length = 50
axle_track = 144

#mm/s
absolute_min_speed = 10 #smallest speed the motors will function at
drive_speed = 500
turn_speed = 250

#ms
dt=20
class Robot:
    def __init__(self, wheel_base, wheel_radius, left_motor: Motor, right_motor: Motor):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.pos = Pose2d(0,0,0)
        self.x = 0
        self.y = 0
        self.theta = 0
    def updatePos(self):
        averageSpeed = (left_motor.speed() + right_motor.speed())/2
        angle = GyroSensor.angle() * pi / 180
        self.x = self.x + averageSpeed * dt * cos(angle)
        self.y = self.y + averageSpeed * dt * sin(angle)
        self.theta = angle
        self.pos = Pose2d(self.x,self.y,self.theta)

class Pose2d:
    def __init__(self, xPos, yPos, theta):
        self.xPos = xPos
        self.yPos = yPos
        self.theta = theta
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
        position = position.updatePos(position)
        
    left_motor.stop()
    right_motor.stop()

    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
    return position

def turn(degrees, speed, position: Pose2d) -> Pose2d:
    gyro_sensor.reset_angle(0)
        
    while abs(gyro_sensor.angle()) <= abs(degrees) - 20:
        turn_speed_ratio = 0.75 + (degrees - gyro_sensor.angle()) / degrees

        print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

        left_motor.run(-turn_speed_ratio * speed * sign(gyro_sensor.angle() - degrees))
        right_motor.run(turn_speed_ratio * speed * sign(gyro_sensor.angle() - degrees))
        position = position.updatePos(position)
    
    align_angle(degrees, position)
    align_angle(degrees, position)
    
    wait(100)
    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
    return position

def drive(distance, speed, position: Pose2d) -> Pose2d:
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
        position = position.updatePos(position)

    align_angle(0, position)
    align_angle(0, position)

    wait(100)

    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

    return position


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