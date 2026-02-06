#!/usr/bin/env pybricks-micropython
from math import sin, cos, pi, atan2, radians, hypot, copysign
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait

# --- CLASSES ---

class Pose2d:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class Vector2d:
    def __init__(self, init_pos: Pose2d, final_pos: Pose2d):
        self.deltaX = final_pos.x - init_pos.x
        self.deltaY = final_pos.y - init_pos.y
        self.magnitude = hypot(self.deltaX, self.deltaY)
        self.angle = atan2(self.deltaY, self.deltaX) * 180 / pi

class Robot:
    def __init__(self, wheel_diam, axle_track, l_motor: Motor, r_motor: Motor, gyro: GyroSensor):
        self.l_motor = l_motor
        self.r_motor = r_motor
        self.gyro = gyro
        self.wheel_diam = wheel_diam
        self.wheel_circ = wheel_diam * pi
        self.axle_track = axle_track

        self.l_motor.reset_angle(0)
        self.r_motor.reset_angle(0)
        self.gyro.reset_angle(0)

        self.pos = Pose2d(0, 0, 0)
        self.prev_left = 0
        self.prev_right = 0
        self.start_angle = 0
        self.rel_dist = 0
        self.rel_angle = 0
        self.speed = 0

    def update_pos(self):
        cur_l = self.l_motor.angle()
        cur_r = self.r_motor.angle()
        cur_gyro = self.gyro.angle()

        delta_l = (cur_l - self.prev_left) / 360.0 * self.wheel_circ
        delta_r = (cur_r - self.prev_right) / 360.0 * self.wheel_circ
        dist_center = (delta_l + delta_r) / 2.0

        self.prev_left = cur_l
        self.prev_right = cur_r

        prev_rad = radians(self.pos.theta)
        self.pos.theta = cur_gyro
        new_rad = radians(self.pos.theta)

        avg_theta = (prev_rad + new_rad) / 2.0

        self.pos.x += dist_center * cos(avg_theta)
        self.pos.y += dist_center * sin(avg_theta)

        self.rel_angle = self.pos.theta - self.start_angle
        self.speed = self.wheel_circ * (self.l_motor.speed() + self.r_motor.speed()) / 2

    def stop(self):
        self.l_motor.brake()
        self.r_motor.brake()

    def align_abs(self, target_angle):
        wait(50)
        while self.pos.theta != target_angle:
            self.update_pos()
            self.l_motor.run(round(MIN_SPEED * copysign(1,target_angle - self.pos.theta)))
            self.r_motor.run(round(-MIN_SPEED * copysign(1, target_angle - self.pos.theta)))
            wait(DT)
            self.l_motor.brake()
            self.r_motor.brake()
            print(str(self.pos.theta) + " LEFT: " + str(self.l_motor.speed()) + " RIGHT: " + str(self.r_motor.speed()))

    def turn_abs(self, target_angle, speed):
        while abs(self.pos.theta) <= abs(target_angle) - 20:
            self.update_pos()
            turn_speed_ratio = 0.75 + (target_angle - self.pos.theta) / target_angle
            turn_speed = round(turn_speed_ratio * speed * copysign(1,self.pos.theta - target_angle))
            self.l_motor.run(-turn_speed)
            self.r_motor.run(turn_speed)
            wait(DT)

        self.align_abs(target_angle)
        self.align_abs(target_angle)

        wait(100)

        print(str(self.pos.theta) + " LEFT: " + str(self.l_motor.speed()) + " RIGHT: " + str(self.r_motor.speed()))

    def turn_rel(self, degrees, speed):
        self.start_angle = self.pos.theta
        target = self.start_angle + degrees
        self.turn_abs(target, speed)

    def drive_abs(self, target_pose: Pose2d, speed):
        distance = Vector2d(self.pos, target_pose)
        initial_distance = distance.magnitude
        pos_neg = copysign(1,cos(distance.angle))
        while distance.magnitude>5:
            avg_encoder_value = abs(initial_distance-distance.magnitude)
            position_ratio = avg_encoder_value / (distance.magnitude * self.wheel_circ)
            drive_speed_ratio = 1 - ((2 * position_ratio - pos_neg) ** 6)
            drive_speed = pos_neg * speed * (drive_speed_ratio + 0.2)
            angle = distance.angle - self.pos.theta
            if self.speed < 20:
                self.l_motor.run(drive_speed - angle * 5)
                self.r_motor.run(drive_speed + angle * 5)
            elif distance.magnitude < 10:
                self.l_motor.run(drive_speed - angle * 5)
                self.r_motor.run(drive_speed + angle * 5)
            else:
                self.l_motor.run(pos_neg * 1.1 * speed - angle * 5)
                self.r_motor.run(pos_neg * 1.1 * speed + angle * 5)
            self.update_pos()
            distance = Vector2d(self.pos, target_pose)

    def drive_to(self, x, y, speed):
        self.drive_abs(Pose2d(x, y, 0), speed)

    def drive_rel(self, distance_cm, speed):
        target_dist_mm = distance_cm * 10
        start_l = self.l_motor.angle()
        start_r = self.r_motor.angle()
        target_rotations = target_dist_mm / self.wheel_circ
        target_degrees = target_rotations * 360

        # Maintain starting heading
        target_heading = self.pos.theta

        traveled = 0
        direction = copysign(1, distance_cm)

        while abs(traveled) < abs(target_degrees): # constant at the end is used to offset error
            self.update_pos()
            traveled = ((self.l_motor.angle() - start_l) + (self.r_motor.angle() - start_r)) / 2
            remaining = abs(target_degrees) - abs(traveled)
            position_ratio = traveled / target_degrees
            drive_speed_ratio = 1 - ((2 * position_ratio - direction) ** 6)
            drive_speed = direction * speed * (drive_speed_ratio + 0.2)
            if traveled < 209:
                self.l_motor.run(drive_speed - self.rel_angle * 5)
                self.r_motor.run(drive_speed + self.rel_angle * 5)
            elif remaining < 209:
                self.l_motor.run(drive_speed - self.rel_angle * 5)
                self.r_motor.run(drive_speed + self.rel_angle * 5)
            else:
                self.l_motor.run(direction * 1.1 * speed - self.rel_angle * 5)
                self.r_motor.run(direction * 1.1 * speed + self.rel_angle * 5)
            wait(DT)

        left_motor.brake()
        right_motor.brake()

        self.align_abs(target_heading)
        self.align_abs(target_heading)

        wait(100)

        print(str(self.pos.theta) + " LEFT: " + str(self.l_motor.speed()) + " RIGHT: " + str(self.r_motor.speed()))

    def square(self, side_length_cm, speed):
        for _ in range(4):
            self.drive_rel(side_length_cm, speed)
            self.turn_rel(90, speed / 2)

# --- SETUP & CONSTANTS ---

ev3 = EV3Brick()
DT = 20  # milliseconds loop time

# Hardware Init
try:
    left_motor = Motor(Port.A)
    right_motor = Motor(Port.D)
    gyro_sensor = GyroSensor(Port.S1)
except Exception as e:
    print(f"Hardware Error: {e}")
    wait(2000)
    raise

MIN_SPEED = 10
DRIVE_SPEED = 500 #mm/s
TURN_SPEED = 250 #mm/s
# Robot Physical Constants (mm)
WHEEL_DIAMETER_MM = 66.5 # Approx EV3 standard, adjust as needed (orig was 20.9cm circ -> ~66.5mm diam)
AXLE_TRACK_MM = 144

# Initialize
robot = Robot(WHEEL_DIAMETER_MM, AXLE_TRACK_MM, left_motor, right_motor, gyro_sensor)

# --- MAIN EXECUTION ---

# Calibration
print("Calibrating...")
gyro_sensor.speed() # switching modes resets drift on some firmwares
gyro_sensor.angle()
wait(500)
print("Go.")

# Action
robot.drive_to(500, 500, 400)
# robot.square(50, 300)