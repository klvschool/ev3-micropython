#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.nxtdevices import LightSensor


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

ev3 = EV3Brick()  # ประกาศตัวแปรหุ่นยนต์
# gyro = GyroSensor(Port.S1)
left_sensor = LightSensor(Port.S4)
right_sensor = LightSensor(Port.S3)
back_left_sensor = LightSensor(Port.S2)
back_right_sensor = LightSensor(Port.S1)
right_motor = Motor(Port.C)
# arm = Motor(Port.A)
left_motor = Motor(Port.B)


robot = DriveBase(left_motor, right_motor,  wheel_diameter=55.5, axle_track=163)
allow_color = [  Color.GREEN, Color.BLUE, Color.RED, Color.WHITE, Color.BROWN, Color.YELLOW ] 
p_color =  [ Color.GREEN]


black_light_list = [30,21,22.4,24,32.6]
white_light_list = [68, 58, 66, 63,
                    68, 58, 66, 63,
                    67, 58, 66, 63,
                    67, 58, 66, 63]

drop_point=0
turn_count = 1
turn_leftlist = [4]
turn_rightlist = [1,2,3,5,6]

def calculate_light(black_light , white_light):
    white = 0
    black = 0
    if len(black_light) != 0 and len(white_light) != 0 :
        for i in white_light:
            white = white + i
        for i in black_light:
            black = black + i
        white = white/len(white_light)
        black = black/len(black_light)
        return (white+black)/2
    else :
        return None

def robot_stop():
    robot.stop()
    right_motor.hold()
    left_motor.hold()

def robot_forword():
    right_motor.run(250)
    left_motor.run(250)
   
def robot_backword():
    right_motor.run(-250)
    left_motor.run(-250)
   
light = calculate_light(black_light_list, white_light_list)
robot.settings(500,300,360,360)
def run():
    if left_sensor.reflection() > light and right_sensor.reflection() > light :
        robot_forword()
    elif left_sensor.reflection() < light and right_sensor.reflection() > light :
        # wait(300)
        robot_stop()
        robot.straight(10)
        robot_stop()
        if left_sensor.reflection() < light and right_sensor.reflection() < light :
            robot_stop()
            robot.straight(-150)
            robot.turn(90)
            robot_stop()
            robot_backword()
            while back_left_sensor.reflection() < light or back_right_sensor.reflection() < light :
                if back_left_sensor.reflection() < light:
                    left_motor.hold()
                elif back_right_sensor.reflection() < light:
                    right_motor.hold()
                break
        else :
            robot_stop()
            robot_backword()
            while left_sensor.reflection() < light :
                right_motor.run_time(-100,500)
                left_motor.run_time(50,500)
                break
    elif left_sensor.reflection() > light and right_sensor.reflection() < light :
        # wait(300)
        robot_stop()
        robot.straight(10)
        robot_stop()
        if left_sensor.reflection() < light and right_sensor.reflection() < light :
            robot_stop()
            robot.straight(-150)
            robot.turn(90)
            robot_stop()
            robot_backword()
            while back_left_sensor.reflection() < light or back_right_sensor.reflection() < light :
                if back_left_sensor.reflection() < light:
                    left_motor.hold()
                elif back_right_sensor.reflection() < light:
                    right_motor.hold()
                break
        else :
            robot_stop()
            robot_backword()
            while right_sensor.reflection() < light :
                left_motor.run_time(-100,500)
                right_motor.run_time(50,500)
                break
    elif left_sensor.reflection() < light and right_sensor.reflection() < light :
        robot_stop()
        robot.turn(90)
        robot_stop()
        robot_backword()
        while back_left_sensor.reflection() < light or back_right_sensor.reflection() < light :
            if back_left_sensor.reflection() < light:
                left_motor.hold()
            elif back_right_sensor.reflection() < light:
                right_motor.hold()
            break
# print(light)
while True:
    run()
    # print('{}, {}, {}, {},'.format(left_sensor.reflection(),right_sensor.reflection(),back_left_sensor.reflection(),back_right_sensor.reflection()))
    # robot.straight(300)
    # robot.turn(90)
    # robot.straight(300)
    # robot.turn(90)
    # robot.straight(300)
    # robot.turn(90)
    # robot.straight(300)
    # robot.turn(90)