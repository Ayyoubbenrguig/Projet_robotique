#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

ev3 = EV3Brick()

MOTOR_PORT = Port.A
ULTRASONIC_PORT = Port.S1
COLOR_PORT = Port.S4

MIN_DISTANCE = 60
MOTOR_SPEED = 400
MOTOR_RUN_TIME = 1000
ultrasonic = UltrasonicSensor(ULTRASONIC_PORT)
color_sensor = ColorSensor(COLOR_PORT)
arm_motor = Motor(MOTOR_PORT)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2
DRIVE_SPEED = 200
PROPORTIONAL_GAIN = 1.2

def lift_arm():
    arm_motor.run_time(speed=MOTOR_SPEED, time=MOTOR_RUN_TIME)

def lower_arm():
    arm_motor.run_time(speed=-MOTOR_SPEED+MOTOR_SPEED*0.3, time=MOTOR_RUN_TIME)
while True:
    # Calculate the deviation from the threshold.
    deviation = color_sensor.reflection() - threshold
    # Calculate the turn rate.
    turn_rate = PROPORTIONAL_GAIN * deviation

    # Set the drive base speed and turn rate.
    robot.drive(DRIVE_SPEED, turn_rate)

    # Check for an object in front of the robot using the ultrasonic sensor.
    distance = ultrasonic.distance()


    if distance < MIN_DISTANCE:
        robot.drive(0,0)
        print("Object found!")
        lift_arm()
        wait(1000)
        lower_arm()
        break
    else:
        print("Incompatible object!")

# You can wait for a short time or do other things in this loop.