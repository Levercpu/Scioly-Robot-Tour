#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch, DataLog

'''
4 SECONDS BETWEEN RUNNING CODE AND ROBOT MOVING

TO-DO:
- test drive and turn methods, shouldn't reset gyro
- test faster drive and turn speeds
- test go_to()
'''

position = [1, 1]

#objects
ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
gyro_sensor = GyroSensor(Port.S1)
timer = StopWatch()

#constants
wheel_circum = 20.9 #cm
min_speed = 10 #mm/s
drive_speed = 750 #mm/s
turn_speed = 250 #mm/s
square_side = 25 #cm


# functions
def sign(x):
    if (x > 0):
        return 1
    elif (x == 0):
        return 0
    else:
        return -1

def filtered_gyro():
    return gyro_sensor.angle() % 360

def align_angle(target_angle):
    wait(50)

    while gyro_sensor.angle() != target_angle:
        print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

        left_motor.run(min_speed * sign(target_angle - gyro_sensor.angle()))
        right_motor.run(-min_speed * sign(target_angle - gyro_sensor.angle()))
        
    left_motor.brake()
    right_motor.brake()

    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

def turn(degrees, speed = turn_speed):
    initial_angle = gyro_sensor.angle()
    timer.reset()

    if(sign(degrees) == 1):
        offset = 20
    elif(sign(degrees) == -1):
        offset = 35
        
    while abs(gyro_sensor.angle() - initial_angle) <= abs(degrees) - offset:
        turn_speed_ratio = 0.75 + (degrees - (gyro_sensor.angle() - initial_angle)) / degrees
        turn_speed = turn_speed_ratio * speed * sign(gyro_sensor.angle() - initial_angle - degrees)

        print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

        left_motor.run(-turn_speed)
        right_motor.run(turn_speed)
    
    align_angle(initial_angle + degrees)
    
    wait(100)

    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
    print(timer.time())


def drive(distance, speed = drive_speed):
    initial_angle = gyro_sensor.angle()
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    timer.reset()

    pos_neg = sign(distance)

    while abs((left_motor.angle() + right_motor.angle()) / 2) < abs(distance) * wheel_circum - 70: # 70 for 750 speed, 25 for 500
        avg_encoder_value = abs((left_motor.angle() + right_motor.angle()) / 2)
        position_ratio = avg_encoder_value / (distance * wheel_circum)
        drive_speed_ratio = 1 - ((2 * position_ratio - pos_neg) ** 6)
        drive_speed = pos_neg * speed * (drive_speed_ratio + 0.2)

        if (avg_encoder_value < 209):
            print("ACC " + str(gyro_sensor.angle() - initial_angle) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
            left_motor.run(drive_speed - (gyro_sensor.angle() - initial_angle) * speed / 40)
            right_motor.run(drive_speed + (gyro_sensor.angle() - initial_angle) * speed / 40)
        elif (avg_encoder_value > (abs(distance) - 10) * 20.9):
            print("DEC " + str(gyro_sensor.angle() - initial_angle) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
            left_motor.run(drive_speed - (gyro_sensor.angle() - initial_angle) * speed / 40)
            right_motor.run(drive_speed + (gyro_sensor.angle() - initial_angle) * speed / 40)
        else:
            print("MAX " + str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
            left_motor.run(pos_neg * 1.1 * speed - (gyro_sensor.angle() - initial_angle) * speed / 40)
            right_motor.run(pos_neg * 1.1 * speed + (gyro_sensor.angle() - initial_angle) * speed / 40)

    left_motor.brake()
    right_motor.brake()

    align_angle(initial_angle)

    wait(100)

    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
    print(timer.time())


def square(drive_distance, turn_angle, drive_speed = drive_speed, turn_speed = turn_speed):
    initial_angle = gyro_sensor.angle()
    
    drive(drive_distance, drive_speed)
    turn(turn_angle, turn_speed)
    drive(drive_distance, drive_speed)
    turn(turn_angle, turn_speed)
    drive(drive_distance, drive_speed)
    turn(turn_angle, turn_speed)
    drive(drive_distance, drive_speed)
    turn(turn_angle, turn_speed)

    print("Angle difference: " + str((gyro_sensor.angle() - initial_angle) % 360))

def check_gyro_drift():
    gyro_sensor.reset_angle(0)

    wait(5000)

    print("Gyro Drift per Second: " + str(gyro_sensor.angle() / 5))

def go_to(x_coord, y_coord, drive_speed = drive_speed, turn_speed = turn_speed):
    ver_distance = square_side * (y_coord - position[1])
    hor_distance = square_side * (x_coord - position[0])

    initial_angle = filtered_gyro()

    if(ver_distance != 0):
        target = 90 - sign(ver_distance + hor_distance) * 90
    elif(hor_distance != 0):
        target = 180 - sign(ver_distance + hor_distance) * 90

    calc_turn = (target - initial_angle) % 360

    if calc_turn > 180:
        calc_turn -= 360

    if(abs(calc_turn) == 180):
        drive(-1 * abs(ver_distance + hor_distance), drive_speed)
        align_angle(initial_angle)
    else:
        turn(calc_turn, turn_speed)
        align_angle(target)
        drive(abs(ver_distance + hor_distance), drive_speed)

    

    position[0] = x_coord
    position[1] = y_coord 


go_to(2, 1)
go_to(2, 2)