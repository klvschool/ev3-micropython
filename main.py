#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog

from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.tools import wait
from pybricks.media.ev3dev import Font
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()  # ประกาศตัวแปรหุ่นยนต์
gyro = GyroSensor(Port.S1)
left_sensor = ColorSensor(Port.S3)
right_sensor = ColorSensor(Port.S2)
# mid_sensor = ColorSensor(Port.S1)
right_motor = Motor(Port.C)
arm = Motor(Port.A)
left_motor = Motor(Port.B)
gearRatio = 24/8
# robot = DriveBase(left_motor, right_motor, wheel_diameter=55, axle_track=180)

# Write your program here.


# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=164)

allow_color = [Color.GREEN, Color.RED, Color.BLUE, Color.YELLOW]
p_color = [Color.GREEN, Color.RED, Color.BLUE, Color.YELLOW]


def forword():
    right_motor.run(250)
    left_motor.run(250)


def backword():
    right_motor.run(-250)
    left_motor.run(-250)


def robot_stop():
    robot.stop()
    right_motor.hold()
    left_motor.hold()


def drop_box(count_box=0):
    count = 0
    for i in range(count_box):
        arm.reset_angle(0)
        
        arm.run_angle(300, 125)
        wait(1000)
        arm.run_until_stalled(200, then=Stop.COAST, duty_limit=50)
        print('Stall Motor')
        arm.run_angle(300, -125)


def robot_forword():
    correction = (0 - gyro.angle())*3
    robot.drive(250, correction)


def robot_turn(angle=0):
    # correction = (0 - gyro.angle())*3
    gyro.reset_angle(0)
    robot.turn(angle)
    error_angle = angle - gyro.angle()
    print('error_angle', error_angle)
    while error_angle != 0:
        if gyro.angle() < angle:
            print('gyro :{} gyro.angle() < angle :{}'.format(gyro.angle(),gyro.angle() < angle))
            wait(300)
            robot.turn(error_angle)
            # continue
        elif gyro.angle() > angle:
            print('gyro :{} gyro.angle() < angle :{}'.format(gyro.angle(),gyro.angle() < angle))
            wait(300)
            robot.turn(error_angle)
            # continue
        wait(300)
        error_angle = angle - gyro.angle()
        # gyro.reset_angle(0)
    # print('gyro :{} '.format(gyro.angle()))


def drive_box():
    # robot.reset()
    while robot.distance() < 300 :
        if (right_sensor.color() not in allow_color and left_sensor.color() not in allow_color and (right_sensor.color() == left_sensor.color())):
            print('Right :{r} , Left :{l}'.format(
                r=right_sensor.color(), l=left_sensor.color()))
            robot_forword()
        elif (right_sensor.color() in p_color and left_sensor.color() in p_color and (right_sensor.color() == left_sensor.color())):
            print('Right :{r} , Left :{l}'.format(
                r=right_sensor.color(), l=left_sensor.color()))
            # robot_stop()
            robot.straight(140)
            robot_stop()
            if (right_sensor.color() == Color.BLACK and left_sensor.color() == Color.BLACK):
                print('drop')
                robot.straight(-80)
                robot.turn(180)
                # robot.straight(130)
                gyro.reset_angle(0)
                robot_stop()
                drop_box()
                break
        elif (right_sensor.color() == Color.BLACK or left_sensor.color() == Color.BLACK):

            # if(right_sensor.color() == Color.BLACK ):
            #         print('r black')
            #         robot_stop()
            #         if (left_sensor.color() != Color.BLACK ):
            #                 left_motor.run(250)

            # elif(left_sensor.color() == Color.BLACK ):
            #         print('l black')
            #         robot_stop()
            #         if (right_sensor.color() != Color.BLACK ):
            #                 right_motor.run(250)

            if (right_sensor.color() == Color.BLACK and left_sensor.color() == Color.BLACK):
                print('RL black')
                robot_stop()
                robot.straight(-80)
                robot_turn(90)

            
        


robot_stop()
arm.reset_angle(0)
robot.reset()
gyro.reset_angle(0)
robot.settings(300, 200, 300, 200)
# print("Speed: ", gyro.speed())
while True:

        
    drop_box()
    

    # wait(10)
    # forward()
    # print(gyro.angle())
    # break
    # print(left_sensor.color(), ' : ', right_sensor.color())


# data = DataLog('robot distant', name='my_log_file', timestamp=False, extension='csv')

# robot.reset()
# robot.drive(1000,0)
# while True :
    # ev3.screen.print(ev3.battery.voltage())
    # robot.drive(100,100)

    # data.log(robot.distance())
    # wait(100)
    # ev3.light.on(Color.BLUE)
    # while robot.distance() == 300:
    # robot.stop()
    # left_motor.brake()
    # robot.straight(-300)
    # right_motor.brake()
