#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Color, Button, Stop
from pybricks.tools import wait, StopWatch
import math

WHEEL_DIA = 56.0
TRACK_WIDTH = 115.0
DEG_PER_MM = 360.0 / (math.pi * WHEEL_DIA)

PID_DIST = [1.5, 0.02, 4.0]  
PID_HEAD = [3.0, 0.05, 8.0]  

GLOBAL_SPEED_FACTOR = 1.0
ACCEL = 3.0
MIN_PWR = 10

# --- SETUP ---
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
gyro = GyroSensor(Port.S1)
timer = StopWatch()

def calibrate_gyro():
    ev3.light.on(Color.ORANGE)
    print("Calibrating...")
    gyro.reset_angle(0)
    while True:
        start_angle = gyro.angle()
        wait(500)
        if gyro.angle() == start_angle: 
            break
        gyro.reset_angle(0)
    ev3.speaker.beep()
    ev3.light.on(Color.GREEN)

def move(target_mm, target_heading, max_speed):

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    
    target_deg = target_mm * DEG_PER_MM
    eff_max_speed = max_speed * GLOBAL_SPEED_FACTOR
    

    sum_err_dist, sum_err_head = 0, 0
    prev_err_dist, prev_err_head = 0, 0
    current_speed_limit = 0

    while True:

        cur_dist = (left_motor.angle() + right_motor.angle()) / 2.0
        cur_head = gyro.angle()
        

        err_dist = target_deg - cur_dist
        err_head = target_heading - cur_head
        

        vel = abs(left_motor.speed()) + abs(right_motor.speed())
        if abs(err_dist) < 10 and abs(err_head) < 2 and vel < 10:
            break

        if current_speed_limit < eff_max_speed: 
            current_speed_limit += ACCEL

        if abs(err_dist) < 200: sum_err_dist += err_dist 
        else: sum_err_dist = 0
            
        P_d = err_dist * PID_DIST[0]
        I_d = sum_err_dist * PID_DIST[1]
        D_d = (err_dist - prev_err_dist) * PID_DIST[2]
        out_dist = P_d + I_d + D_d
        prev_err_dist = err_dist

        out_dist = max(min(out_dist, current_speed_limit), -current_speed_limit)

        if abs(err_head) < 15: sum_err_head += err_head
        else: sum_err_head = 0
            
        P_h = err_head * PID_HEAD[0]
        I_h = sum_err_head * PID_HEAD[1]
        D_h = (err_head - prev_err_head) * PID_HEAD[2]
        out_head = P_h + I_h + D_h
        prev_err_head = err_head

        if target_mm == 0: out_dist = 0

        if out_dist > 0: out_dist += MIN_PWR
        if out_dist < 0: out_dist -= MIN_PWR
        if target_mm == 0:
             if out_head > 0: out_head += MIN_PWR
             elif out_head < 0: out_head -= MIN_PWR

        left_motor.dc(max(min(out_dist + out_head, 100), -100))
        right_motor.dc(max(min(out_dist - out_head, 100), -100))
        
        wait(10)

    left_motor.brake()
    right_motor.brake()
    wait(200)


calibrate_gyro()


while not Button.CENTER in ev3.buttons.pressed():
    wait(10)
wait(500)
ev3.speaker.beep()
timer.reset()


path = [
    [500, 0, 80],
    [0, 90, 50],
    [300, 90, 80],
    [0, 180, 50],
    [800, 180, 100],
    [0, 360, 50],
    [-100, 360, 60]
]

for step in path:
    move(step[0], step[1], step[2])

final_time = timer.time() / 1000.0
print("Run Complete: " + str(final_time) + "s")
ev3.speaker.beep(800, 1000)