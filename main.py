#!/usr/bin/env pybricks-micropython
from math import sin, cos, pi, atan2, radians, sqrt, copysign
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port
from pybricks.tools import wait
def difference_deg(a_2, a_1):
    diff = a_2-a_1+180
    while diff>360:
        diff-=360
    while diff<0:
        diff+=360
    diff-=180
    return diff
# --- CLASSES ---

class Pose2d:
    def __init__(self, x_cm, y_cm, theta_deg):
        self.x_cm = x_cm
        self.y_cm = y_cm
        self.theta_deg = theta_deg

class Vector2d:
    def __init__(self, init_pos: Pose2d, final_pos: Pose2d):
        self.deltaX_cm = final_pos.x_cm - init_pos.x_cm
        self.deltaY_cm = final_pos.y_cm - init_pos.y_cm
        self.magnitude = sqrt(self.deltaX_cm**2+ self.deltaY_cm**2)
        self.angle = atan2(self.deltaY_cm, self.deltaX_cm)
        while self.angle<0:
            self.angle+=2*pi
        while self.angle>2*pi:
            self.angle-=2*pi

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
        self.start_angle_deg = 0
        self.rel_dist = 0
        self.rel_angle_deg = 0
        self.speed = 0
        self.count=0
    def print_all(self):
        print("x: " + str(self.pos.x_cm) + ", y: " + str(self.pos.y_cm) + ", angle: " + str(self.pos.theta_deg) + ", speed: " + str(self.speed))
    def update_pos(self):
        cur_l = self.l_motor.angle()
        cur_r = self.r_motor.angle()
        cur_gyro = -self.gyro.angle()

        delta_l = (cur_l - self.prev_left) / 360.0 * self.wheel_circ / 10.0
        delta_r = (cur_r - self.prev_right) / 360.0 * self.wheel_circ / 10.0
        dist_center_cm = (delta_l + delta_r) / 2.0

        self.prev_left = cur_l
        self.prev_right = cur_r

        prev_rad = radians(self.pos.theta_deg)
        while cur_gyro>360:
            cur_gyro-=360
        while cur_gyro<0:
            cur_gyro+=360
        self.pos.theta_deg = cur_gyro
        new_rad = radians(self.pos.theta_deg)

        avg_theta_rad = (prev_rad + new_rad) / 2.0
        delta_theta_rad = prev_rad-new_rad

        self.pos.x_cm += dist_center_cm * cos(avg_theta_rad)
        self.pos.y_cm += dist_center_cm * sin(avg_theta_rad)
        # if (delta_l!=delta_r):
        #     r_a = delta_l*(AXLE_TRACK_MM/10.0)/(delta_l-delta_r)
        #     r_b = delta_l*(AXLE_TRACK_MM/10.0)/(delta_l-delta_r)
        #     r_x = (r_a + r_b) / 2.0
        #     A=(delta_l-delta_r)/(AXLE_TRACK_MM/10.0)
        #     x,y = sin(A)*r_x,(1-cos(A))*r_x
        #     c, s = cos(avg_theta_rad), sin(avg_theta_rad)
        #     self.pos.x_cm += x*c-y*s
        #     self.pos.y_cm += x*s+y*c
        # else:
        #     self.pos.x_cm += dist_center_cm * cos(avg_theta_rad)
        #     self.pos.y_cm += dist_center_cm * sin(avg_theta_rad)

        self.rel_angle_deg = difference_deg(self.pos.theta_deg, self.start_angle_deg)
        self.speed = self.wheel_circ * (self.l_motor.speed() + self.r_motor.speed()) / 720
        self.count += 1
        if self.count % 20 == 0:
            self.print_all()

    def stop(self):
        self.l_motor.brake()
        self.r_motor.brake()

    def align_abs(self, target_angle_deg):
        print("aligning to angle", target_angle_deg)
        target_angle_deg = round(target_angle_deg)
        while self.pos.theta_deg != target_angle_deg:
            self.update_pos()
            self.l_motor.run(-round(MIN_SPEED * copysign(1,difference_deg(target_angle_deg, self.pos.theta_deg))))
            self.r_motor.run(round(MIN_SPEED * copysign(1, difference_deg(target_angle_deg, self.pos.theta_deg))))
            wait(DT)
        self.stop()
        self.print_all()
        print("finished aligning")

    def turn_abs(self, target_angle_deg, speed):
        target_angle_deg = round(target_angle_deg)
        print("turning to", target_angle_deg)
        while abs(difference_deg(target_angle_deg,self.pos.theta_deg)) > 5:
            self.update_pos()
            turn_speed_ratio = 0.75 + abs(difference_deg(target_angle_deg, self.pos.theta_deg)) / 90
            turn_speed = round(turn_speed_ratio * speed * copysign(1,difference_deg(self.pos.theta_deg,target_angle_deg)))
            self.l_motor.run(turn_speed)
            self.r_motor.run(-turn_speed)
            wait(DT)
        self.stop()
        print("finished turning")

        self.print_all()
        

    def turn_rel(self, target_angle_deg, speed):
        self.start_angle_deg = self.pos.theta_deg
        target_deg = self.start_angle_deg + target_angle_deg
        self.turn_abs(target_deg, speed)

    def drive_abs(self, target_pose: Pose2d, speed):
        print("driving to", target_pose.x_cm, target_pose.y_cm)
        distance = Vector2d(self.pos, target_pose)
        if abs(difference_deg(distance.angle*180/pi,self.pos.theta_deg))>90:
            self.turn_abs(distance.angle*180/pi-180, TURN_SPEED)
        else:
            self.turn_abs(distance.angle*180/pi, TURN_SPEED)
        initial_distance = distance.magnitude
        pos_neg = copysign(1,cos(difference_deg(distance.angle,self.pos.theta_deg*pi/180)))
        if pos_neg==1:
            print("forward")
        else:
            print("backward")
        while distance.magnitude>5:
            self.update_pos()
            distance = Vector2d(self.pos, target_pose)
            avg_encoder_value_pos = abs(initial_distance-distance.magnitude)
            position_ratio = avg_encoder_value_pos / (distance.magnitude * self.wheel_circ)
            drive_speed_ratio = 1 - ((2 * position_ratio - 1) ** 6)
            drive_speed = pos_neg * speed * (drive_speed_ratio + 0.2)
            angle_deg = difference_deg(distance.angle*180/pi, self.pos.theta_deg)
            if angle_deg>90:
                angle_deg-=180
            elif angle_deg<-90:
                angle_deg+=180
            if self.speed < 20:
                self.l_motor.run(drive_speed - angle_deg * 5)
                self.r_motor.run(drive_speed + angle_deg * 5)
            elif distance.magnitude < 10:
                self.l_motor.run(drive_speed - angle_deg * 5)
                self.r_motor.run(drive_speed + angle_deg * 5)
            else:
                self.l_motor.run(pos_neg * 1.1 * speed - angle_deg * 5)
                self.r_motor.run(pos_neg * 1.1 * speed + angle_deg * 5)
            wait(DT)
        self.stop()
        self.print_all()
        print("finished driving")
    def drive_to(self, x_cm, y_cm, speed):
        self.drive_abs(Pose2d(x_cm,y_cm,0),speed)
    def final_drive_align(self, x, y):
        print("final align")
        target_pose = Pose2d(x,y,0)
        distance = Vector2d(self.pos, target_pose)
        self.turn_abs(distance.angle*180/pi, TURN_SPEED)
        self.align_abs(distance.angle*180/pi)
        self.align_abs(distance.angle*180/pi)
        pos_neg = copysign(1,cos(difference_deg(distance.angle,self.pos.theta_deg*pi/180)))
        while distance.magnitude>1:
            self.update_pos()
            distance = Vector2d(self.pos, target_pose)
            pos_neg = copysign(1,cos(difference_deg(distance.angle,self.pos.theta_deg*pi/180)))
            self.l_motor.run(round(MIN_SPEED*2*pos_neg))
            self.r_motor.run(round(MIN_SPEED*2*pos_neg))
            wait(DT)
        while distance.magnitude>0.05:
            self.update_pos()
            distance = Vector2d(self.pos, target_pose)
            pos_neg = copysign(1,cos(difference_deg(distance.angle,self.pos.theta_deg*pi/180)))
            self.l_motor.run(round(MIN_SPEED*pos_neg))
            self.r_motor.run(round(MIN_SPEED*pos_neg))
            wait(DT)
        self.stop()
        self.print_all()
        print("finished final align")

    def drive_rel(self, distance_cm, speed):
        target_dist_mm = distance_cm * 10
        start_l = self.l_motor.angle()
        start_r = self.r_motor.angle()
        target_rotations = target_dist_mm / self.wheel_circ
        target_degrees = target_rotations * 360

        # Maintain the starting heading
        target_heading_deg = self.pos.theta_deg
        self.start_angle_deg = target_heading_deg
        traveled = 0
        direction = copysign(1, distance_cm)

        while abs(traveled) < abs(target_degrees): # constant at the end is used to offset the error
            self.update_pos()
            traveled = ((self.l_motor.angle() - start_l) + (self.r_motor.angle() - start_r)) / 2
            remaining = abs(target_degrees) - abs(traveled)
            position_ratio = traveled / target_degrees
            drive_speed_ratio = 1 - ((2 * position_ratio - direction) ** 6)
            drive_speed = direction * speed * (drive_speed_ratio + 0.2)
            if traveled < 209:
                self.l_motor.run(drive_speed + self.rel_angle_deg * 1)
                self.r_motor.run(drive_speed - self.rel_angle_deg * 1)
            elif remaining < 209:
                self.l_motor.run(drive_speed + self.rel_angle_deg * 1)
                self.r_motor.run(drive_speed - self.rel_angle_deg * 1)
            else:
                self.l_motor.run(direction * 1.1 * speed + self.rel_angle_deg * 1)
                self.r_motor.run(direction * 1.1 * speed - self.rel_angle_deg * 1)
            wait(DT)

        left_motor.brake()
        right_motor.brake()

        self.align_abs(target_heading_deg)
        self.align_abs(target_heading_deg)


        self.print_all()

    def square_rel(self, side_length_cm, speed):
        for _ in range(4):
            self.drive_rel(side_length_cm, speed)
            self.turn_rel(90, speed / 2)
    def square_abs(self, side_length_cm, speed):
        self.drive_to(0, side_length_cm, speed)
        self.drive_to(-side_length_cm, side_length_cm, speed)
        self.drive_to(-side_length_cm, 0, speed)
        self.drive_to(0, 0, speed)
        self.final_drive_align(0,0)

def run_track(robot2:Robot):
    robot2.drive_to(77, 0, DRIVE_SPEED)
    robot2.drive_to(27, 100, DRIVE_SPEED)
    robot2.drive_to(77, 0, DRIVE_SPEED)
    robot2.drive_to(27, -100, DRIVE_SPEED)
    robot2.drive_to(77, -100, DRIVE_SPEED)
    robot2.drive_to(27, -100, DRIVE_SPEED)
    robot2.drive_to(50, -50, DRIVE_SPEED)
    robot2.drive_to(100, -50, DRIVE_SPEED)
    robot2.drive_to(177, -100, DRIVE_SPEED)
    robot2.drive_to(177, 50, DRIVE_SPEED)


# --- SETUP & CONSTANTS ---

ev3 = EV3Brick()
DT = 20  # milliseconds loop time

# Hardware Init

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
gyro_sensor = GyroSensor(Port.S1)

MIN_SPEED = 10
DRIVE_SPEED = 200 #mm/s
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
#robot.drive_to(100,100,DRIVE_SPEED)
#robot.final_drive_align(100,100)
print("Done")
robot.drive_to(-50,-50,400)
# run_Track(robot)