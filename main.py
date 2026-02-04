from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch, DataLog

'''
TO-DO LIST

- test on actual track
- time drives and turns
'''

#objects
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
gyro_sensor = GyroSensor(Port.S1)

#constants
wheel_circum = 20.9 #cm
min_speed = 10 #mm/s
drive_speed = 500 #mm/s
turn_speed = 250 #mm/s


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

        left_motor.run(min_speed * sign(target_angle - gyro_sensor.angle()))
        right_motor.run(-min_speed * sign(target_angle - gyro_sensor.angle()))
        
    left_motor.brake()
    right_motor.brake()

    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

def turn(degrees, speed):
    gyro_sensor.reset_angle(0)
        
    while abs(gyro_sensor.angle()) <= abs(degrees) - 20:
        turn_speed_ratio = 0.75 + (degrees - gyro_sensor.angle()) / degrees
        turn_speed = turn_speed_ratio * speed * sign(gyro_sensor.angle() - degrees)

        print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

        left_motor.run(-turn_speed)
        right_motor.run(turn_speed)
    
    align_angle(degrees)
    align_angle(degrees)
    
    wait(100)

    print(str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))

def drive(distance, speed):
    gyro_sensor.reset_angle(0)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)

    pos_neg = sign(distance)

    while abs((left_motor.angle() + right_motor.angle()) / 2) < abs(distance) * wheel_circum - 40: # constant at the end is used to offset error
        avg_encoder_value = abs((left_motor.angle() + right_motor.angle()) / 2)
        position_ratio = avg_encoder_value / (distance * wheel_circum)
        drive_speed_ratio = 1 - ((2 * position_ratio - pos_neg) ** 6)
        drive_speed = (pos_neg * speed * drive_speed_ratio) + pos_neg * speed / 5

        if (avg_encoder_value < 209):
            print("ACC " + str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
            left_motor.run(drive_speed - gyro_sensor.angle() * 5)
            right_motor.run(drive_speed + gyro_sensor.angle() * 5)
        elif (avg_encoder_value > (abs(distance) - 10) * 20.9):
            print("DEC " + str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
            left_motor.run(drive_speed - gyro_sensor.angle() * 5)
            right_motor.run(drive_speed + gyro_sensor.angle() * 5)
        else:
            print("MAX " + str(gyro_sensor.angle()) + " LEFT: " + str(left_motor.speed()) + " RIGHT: " + str(right_motor.speed()))
            left_motor.run(pos_neg * 1.1 * speed - gyro_sensor.angle() * 5)
            right_motor.run(pos_neg * 1.1 * speed + gyro_sensor.angle() * 5)

    left_motor.brake()
    right_motor.brake()

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


square(-50, 500, -91, 250)