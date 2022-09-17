#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

ev3 = EV3Brick()  # ประกาศตัวแปรหุ่นยนต์
gyro = GyroSensor(Port.S1)
left_sensor = ColorSensor(Port.S3)
right_sensor = ColorSensor(Port.S2)
right_motor = Motor(Port.C)
arm = Motor(Port.A)
left_motor = Motor(Port.B)


robot = DriveBase(left_motor, right_motor,  wheel_diameter=55.5, axle_track=163)
allow_color = [  Color.GREEN, Color.BLUE, Color.RED, Color.WHITE, Color.BROWN, Color.YELLOW ] 
p_color =  [ Color.GREEN]

drop_point=0
turn_count = 1
turn_leftlist = [4]
turn_rightlist = [1,2,3,5,6]

def robot_stop():
    robot.stop()
    right_motor.hold()
    left_motor.hold()

def robot_forword():
    correction = (0 - gyro.angle())*2
    robot.drive(200,correction)

def robot_set():
    def robot_set():
        gyro.reset_angle(0)
        robot.turn(90)
        if (gyro.angle() == 90):
                robot_stop()
        elif (gyro.angle() != 90):
                a = (90 - gyro.angle())
                robot.turn(a)
                gyro.reset_angle(0)    

def foword_white():
    gyro.reset_angle(0)
    if(right_sensor.color() in allow_color and left_sensor.color() in allow_color):
        robot_forword()     


def black_right():
    if (right_sensor.color() == Color.BLACK and  right_sensor.reflection() < 15) or (left_sensor.color() == Color.BLACK):
        robot_stop()
        wait(100)
        robot.straight(-10)
        tune_black()
        robot.straight(-50)
        global turn_count
        if(turn_count in turn_rightlist):
            robot_turn_PID(87)    
            turn_count = turn_count+1                
        elif(turn_count in turn_leftlist):
            robot_turn_PID(-87)  
            turn_count = turn_count+1   
            
def tune_black():
    print('tune_black')
    robot_stop()
    if ((right_sensor.color() == Color.BLACK and  right_sensor.reflection() < 5)and left_sensor.color() != Color.BLACK):
        while (left_sensor.color() != Color.BLACK):
            right_motor.hold()
            left_motor.run(300)
        robot_stop()
    elif ((left_sensor.color() == Color.BLACK and  left_sensor.reflection() < 5) and right_sensor.color() != Color.BLACK):
        while (right_sensor.color() != Color.BLACK):
            right_motor.run(300)
            left_motor.hold()
        robot_stop()

def black_left():
    if (right_sensor.color() == Color.BLACK and left_sensor.color() == Color.BLACK):
        robot_stop()
        robot.straight(-70)
        robot_turn(-90)  

def forword():
    right_motor.run(300)
    left_motor.run(300)
    
def robot_backword():
    right_motor.run(-300)
    left_motor.run(-300)

def check_blackhole_right():
    def check_blackhole():
        robot.straight(50)
        robot.stop()
        
        if  (right_sensor.color()  == Color.BLACK and left_sensor.color() == Color.BLACK):
                robot.straight(250)
                robot_stop()

        elif (right_sensor.color()  == Color.WHITE and left_sensor.color() == Color.WHITE):
                robot.straight(-150)
                robot.turn(90)
                robot.stop()
                forword()
        
def robot_hit():
    arm.run_angle(400,-70)
    arm.run_angle(200,70)  
    wait(500)
    # arm.run_until_stalled(200,Stop.HOLD,20)
    # arm.hold()
def drop_box(count_box=0):
    count = 0
    arm.stop()
    for i in range(count_box):
        arm.reset_angle(0)
        print('Stall Motor : {}'.format(arm.control.stalled()))
        last_angle = 0
        arm.run_angle(300, 125)
        last_angle = arm.angle()
        while arm.control.stalled() == True:
            print('last_angle : {}'.format(last_angle))
            wait(500)
            arm.run_angle(300, -(last_angle))
        arm.run_angle(300, -125)

def robot_stopgreen():
    global drop_point
    if  (right_sensor.color()  == Color.GREEN or left_sensor.color() == Color.GREEN):
        robot_stop()
        robot.straight(140)
        robot_stop()
        if left_sensor.color() == Color.BLACK or right_sensor.color() == Color.BLACK :
            tune_black()
            print('drop')
            robot.straight(-120)
            drop_box(2)
            robot.straight(-120)
            robot_turn(180)
            # robot.straight(130)
            # gyro.reset_angle(0)
            robot_stop()
            drop_point = drop_point + 1
            # exit()


def robot_turn_PID(angle=0):
    # correction = (0 - gyro.angle())*3
    gyro.reset_angle(0)
    # robot.turn(angle)
    PROPORTIONAL_GAIN = 3.8
    INTEGRAL_GAIN = 0.0008
    DERIVATIVE_GAIN = 0.001
    integral = 0
    derivative = 0
    last_error_angle = angle - gyro.angle()
    

    while last_error_angle != 0:
        error_angle = angle - gyro.angle()
        print('error_angle', error_angle)
        integral = integral + error_angle
        derivative = error_angle
        
        turn_rate = PROPORTIONAL_GAIN * error_angle + INTEGRAL_GAIN * integral + DERIVATIVE_GAIN * derivative
        robot.drive(30,turn_rate)
        wait(300)
        error_angle = angle - gyro.angle()
        last_error_angle = error_angle
    gyro.reset_angle(0)

def robot_turn(angle=0):
    # correction = (0 - gyro.angle())*3
    gyro.reset_angle(0)
    robot.turn(angle)
    error_angle = angle - gyro.angle()
    print('error_angle', error_angle)
    while error_angle != 0:
        if gyro.angle() < angle:
            print('gyro :{} gyro.angle() < angle :{}'.format(gyro.angle(),gyro.angle() < angle))
            # wait(300)
            robot.turn(error_angle)
            # continue
        elif gyro.angle() > angle:
            print('gyro :{} gyro.angle() < angle :{}'.format(gyro.angle(),gyro.angle() < angle))
            # wait(300)
            robot.turn(error_angle)
            # continue
        wait(300)
        error_angle = angle - gyro.angle()
        # gyro.reset_angle(0)
    # print('gyro :{} '.format(gyro.angle()))

robot_stop()
arm.reset_angle(0)
robot.settings(300, 200, 300, 200)
while drop_point < 1:
    print('Right :{r} , Left :{l}'.format(r=right_sensor.color(),l=left_sensor.color()))
    # robot.reset()
    # if Button.CENTER in ev3.buttons.pressed():

    foword_white()
    black_right()
    foword_white()
    robot_stopgreen()
    if drop_point == 1:
        robot.straight(150)
        robot_turn_PID(90)
        robot.straight(600)
    foword_white()
    black_right()
    foword_white()
    black_right()
    robot_stopgreen()
    if drop_point == 2:
        robot.straight(150)
    # foword_white()
    # black_left() 
    # foword_white()
    # robot_stopgreen()