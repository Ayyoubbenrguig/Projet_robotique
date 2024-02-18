#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor , UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

line_sensor = ColorSensor(Port.S3)
obstacle_sensor = UltrasonicSensor(Port.S4)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
center_motor = Motor(Port.A)


BLACK = 10
WHITE = 75
threshold = (BLACK + WHITE) / 2

DRIVE_SPEED = 150

PROPORTIONAL_GAIN = 4

def descendre_bras():
    center_motor.run_angle(200, -130, wait=True)

def monter_bras():
    center_motor.run_angle(200, 130, wait=True)

ev3.speaker.beep()

while True:
    if line_sensor.reflection() - threshold < 10 :
        PROPORTIONAL_GAIN = 0.5
    else :
        PROPORTIONAL_GAIN = 4
    
    deviation = line_sensor.reflection() - threshold
    turn_rate = PROPORTIONAL_GAIN * deviation

    robot.drive(DRIVE_SPEED, turn_rate)

    if obstacle_sensor.distance() < 20 :
        descendre_bras()
        center_motor.stop()

        ev3.speaker.beep()
    