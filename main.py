#!/usr/bin/env pybricks-micropython
from math import sin, cos, pi, sqrt, atan
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

wheel_circum = 20.9 #cm
min_speed = 10 #mm/s
drive_speed = 500 #mm/s
turn_speed = 250 #mm/s

#ms
dt=20


class Robot:
    def __init__(self, wheel_base, wheel_circum, left_motor: Motor, right_motor: Motor, gyro: GyroSensor):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.gyro = gyro
        self.wheel_base = wheel_base
        self.wheel_circum = wheel_circum
        self.pos = Pose2d(0,0,0)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.start_left = 0
        self.start_right = 0
        self.rel_distance = 0
        self.rel_angle = 0
        self.start_angle = 0
        self.speed = 0
    def updatePos(self):
        self.speed = (self.left_motor.speed() + self.right_motor.speed())/2 / 360 * wheel_circum
        self.theta = self.gyro.angle()
        angle = self.theta * pi / 180
        self.x = self.x + self.speed * dt * cos(angle)
        self.y = self.y + self.speed * dt * sin(angle)
        self.pos = Pose2d(self.x,self.y,self.theta)
        self.rel_distance = self.distance()
        self.rel_angle = self.theta - self.start_angle
    def align_abs(self, target_angle):
        wait(50)
        while self.theta != target_angle:
            self.updatePos()
            self.left_motor.run(min_speed * sign(target_angle - self.theta))
            self.right_motor.run(-min_speed * sign(target_angle - self.theta))
            wait(dt)  
        self.left_motor.brake()
        self.right_motor.brake()
        print(str(self.theta) + " LEFT: " + str(self.left_motor.speed()) + " RIGHT: " + str(self.right_motor.speed()))
    def align_rel(self, target_angle):
        wait(50)
        while self.rel_angle != target_angle:
            self.updatePos()
            self.left_motor.run(min_speed * sign(target_angle - self.rel_angle))
            self.right_motor.run(-min_speed * sign(target_angle - self.rel_angle))
            wait(dt)  
        self.left_motor.brake()
        self.right_motor.brake()

        print(str(self.theta) + " LEFT: " + str(self.left_motor.speed()) + " RIGHT: " + str(self.right_motor.speed()))
    def turn_abs(self, target_angle, speed):
        while abs(self.theta) <= abs(target_angle) - 20:
            self.updatePos()
            turn_speed_ratio = 0.75 + (target_angle - self.theta) / target_angle
            turn_speed = turn_speed_ratio * speed * sign(self.theta - target_angle)
            self.left_motor.run(-turn_speed)
            self.right_motor.run(turn_speed)
            wait(dt)
        
        self.align_abs(target_angle)
        self.align_abs(target_angle)
        
        wait(100)
        
        print(str(self.theta) + " LEFT: " + str(self.left_motor.speed()) + " RIGHT: " + str(self.right_motor.speed()))
    def turn_rel(self, degrees, speed):
        self.start_angle = 0
        while abs(self.rel_angle) <= abs(degrees) - 20:
            self.updatePos()
            turn_speed_ratio = 0.75 + (degrees - self.rel_angle) / degrees
            turn_speed = turn_speed_ratio * speed * sign(self.rel_angle - degrees)
            self.left_motor.run(-turn_speed)
            self.right_motor.run(turn_speed)
            wait(dt)
        
        self.align_rel(degrees)
        self.align_rel(degrees)
        
        wait(100)

        print(str(self.theta) + " LEFT: " + str(self.left_motor.speed()) + " RIGHT: " + str(self.right_motor.speed()))
    def drive_abs(self, target_pose: Pose2d, speed):
        distance = Vector2d(self.pos, target_pose)
        initial_distance = distance.magnitude
        pos_neg = sign(cos(distance.angle))
        while (distance.magnitude>5):
            avg_encoder_value = abs(initial_distance-distance.magntiude)
            position_ratio = avg_encoder_value / (distance * wheel_circum)
            drive_speed_ratio = 1 - ((2 * position_ratio - pos_neg) ** 6)
            drive_speed = pos_neg * speed * (drive_speed_ratio + 0.2)
            angle = distance.angle - self.theta
            if(self.speed < 20) :
                self.left_motor.run(drive_speed - angle * 5)
                self.right_motor.run(drive_speed + angle * 5)
            elif(distance.magnitude < 10):
                self.left_motor.run(drive_speed - angle * 5)
                self.right_motor.run(drive_speed + angle * 5)
            else: 
                self.left_motor.run(pos_neg * 1.1 * speed - angle * 5)
                self.right_motor.run(pos_neg * 1.1 * speed + angle * 5)
            self.updatePos()
            distance = Vector2d(self.pos, target_pose)
    def drive_to(self, x, y, speed):
        self.drive_abs(Pose2d(x,y,0) ,speed)
    def drive_rel(self, distance, speed):
        self.start_angle = self.theta
        self.start_left = self.left_motor.angle()
        self.start_right = self.right_motor.angle()

        pos_neg = sign(distance)

        while abs(self.rel_distance) < abs(distance) * wheel_circum - 40: # constant at the end is used to offset error
            self.updatePos()
            avg_encoder_value = abs(self.rel_distance)
            position_ratio = avg_encoder_value / (distance * wheel_circum)
            drive_speed_ratio = 1 - ((2 * position_ratio - pos_neg) ** 6)
            drive_speed = pos_neg * speed * (drive_speed_ratio + 0.2)
            if (avg_encoder_value < 209):
                self.left_motor.run(drive_speed - self.rel_angle * 5)
                self.right_motor.run(drive_speed + self.rel_angle * 5)
            elif (avg_encoder_value > (abs(distance) - 10) * 20.9):
                self.left_motor.run(drive_speed - self.rel_angle * 5)
                self.right_motor.run(drive_speed + self.rel_angle * 5)
            else:
                self.left_motor.run(pos_neg * 1.1 * speed - self.rel_angle * 5)
                self.right_motor.run(pos_neg * 1.1 * speed + self.rel_angle * 5)
            wait(dt)

        left_motor.brake()
        right_motor.brake()

        self.align_rel(0)
        self.align_rel(0)

        wait(100)

        print(str(self.theta) + " LEFT: " + str(self.left_motor.speed()) + " RIGHT: " + str(self.right_motor.speed()))
    def distance(self):
        return (self.left_motor.angle()-self.start_left+self.right_motor.angle()-self.start_right)/2
    def square(self,drive_distance, drive_speed, turn_angle, turn_speed):
        self.drive_rel(drive_distance, drive_speed)
        self.turn_rel(turn_angle, turn_speed)
        self.drive_rel(drive_distance, drive_speed)
        self.turn_rel(turn_angle, turn_speed)
        self.drive_rel(drive_distance, drive_speed)
        self.turn_rel(turn_angle, turn_speed)
        self.drive_rel(drive_distance, drive_speed)
        self.turn_rel(turn_angle, turn_speed)

class Pose2d:
    def __init__(self, xPos, yPos, theta):
        self.xPos = xPos
        self.yPos = yPos
        self.theta = theta

class Vector2d:
    def __init__(self, init_pos: Pose2d, final_pos: Pose2d):
        self.deltaX = final_pos.xPos - init_pos.xPos
        self.deltaY = final_pos.yPos - init_pos.yPos
        self.magnitude = sqrt(self.deltaX,self.deltaY)
        self.angle = atan(self.deltaY/self.deltaX) * 180 / pi
# functions

def sign(x):
    if (x > 0):
        return 1
    elif (x == 0):
        return 0
    else:
        return -1

robot = Robot(axle_track, wheel_circum, left_motor, right_motor, gyro_sensor)
robot.drive_to(50, 50, 500)