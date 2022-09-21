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
gyro_sensor = GyroSensor(Port.S1)
left_sensor = ColorSensor(Port.S3)
right_sensor = ColorSensor(Port.S2)
right_motor = Motor(Port.C)
arm = Motor(Port.A)
left_motor = Motor(Port.B)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=163)
allow_color = [Color.BLUE, Color.RED, Color.WHITE, Color.BROWN, Color.YELLOW]
p_color = [Color.GREEN]

drop_point = 0
turn_count = 1
turn_leftlist = [4]
turn_rightlist = [1, 2, 3]

# Initialize the timers.
fall_timer = StopWatch()
single_loop_timer = StopWatch()
control_loop_timer = StopWatch()
action_timer = StopWatch()

GYRO_CALIBRATION_LOOP_COUNT = 100
GYRO_OFFSET_FACTOR = 0.0005
TARGET_LOOP_PERIOD = 15  # ms
ARM_MOTOR_SPEED = 600  # deg/s


gyro_sensor.reset_angle(0)


def a_angle():
    # Calibrate the gyro offset. This makes sure that the robot is perfectly
    # still by making sure that the measured rate does not fluctuate more than
    # 2 deg/s. Gyro drift can cause the rate to be non-zero even when the robot
    # is not moving, so we save that value for use later.
    while True:
        gyro_minimum_rate, gyro_maximum_rate = 440, -440
        gyro_sum = 0
        for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
            gyro_sensor_value = gyro_sensor.angle()
            gyro_sum += gyro_sensor_value
            if gyro_sensor_value > gyro_maximum_rate:
                gyro_maximum_rate = gyro_sensor_value
            if gyro_sensor_value < gyro_minimum_rate:
                gyro_minimum_rate = gyro_sensor_value
            wait(5)
        print('Gyro Rate: {} '.format(gyro_maximum_rate -
              gyro_minimum_rate, gyro_sensor.angle()))
        if gyro_maximum_rate - gyro_minimum_rate < 2:
            angle = gyro_sensor.angle()
            yield angle
            break
    gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT

    while True:
        # calculate robot body angle and speed
        gyro_sensor_value = gyro_sensor.angle()
        gyro_offset *= (1 - GYRO_OFFSET_FACTOR)
        gyro_offset += GYRO_OFFSET_FACTOR * gyro_sensor_value
        robot_body_rate = gyro_sensor_value - gyro_offset
        # Make sure loop time is at least TARGET_LOOP_PERIOD. The output power
        # calculation above depends on having a certain amount of time in each
        # loop.
        if fall_timer.time() > 1000:
            angle = gyro_sensor.angle()
            yield angle
            break

        wait(TARGET_LOOP_PERIOD - single_loop_timer.time())
    print(' Gyro Offset: {} \n Gyro Angle: {} \nrobot_body_rate :{}'.format(
        gyro_offset, gyro_sensor.angle(), robot_body_rate))
    # return gyro_sensor.angle()


def robot_turn_PID(angle=0):
    PROPORTIONAL_GAIN = 1.3
    INTEGRAL_GAIN = 0.0008
    DERIVATIVE_GAIN = 0.001
    integral = 0
    derivative = 0
    
    # left_motor.reset_angle(0)
    # right_motor.reset_angle(0)
    fall_timer.reset()
    
    while True:
        gyro_minimum_rate, gyro_maximum_rate = 440, -440
        gyro_sum = 0
        for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
            gyro_sensor_value = gyro_sensor.angle()
            gyro_sum += gyro_sensor_value
            if gyro_sensor_value > gyro_maximum_rate:
                gyro_maximum_rate = gyro_sensor_value
            if gyro_sensor_value < gyro_minimum_rate:
                gyro_minimum_rate = gyro_sensor_value
            wait(5)
        print('Gyro Rate: {} '.format(gyro_maximum_rate -
              gyro_minimum_rate, gyro_sensor.angle()))
        if gyro_maximum_rate - gyro_minimum_rate < 2:
            break
    gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT

    while True:
        # calculate robot body angle and speed
        gyro_sensor_value = gyro_sensor.angle()
        gyro_offset *= (1 - GYRO_OFFSET_FACTOR)
        gyro_offset += GYRO_OFFSET_FACTOR * gyro_sensor_value
        robot_body_rate = gyro_sensor_value - gyro_offset
        # Make sure loop time is at least TARGET_LOOP_PERIOD. The output power
        # calculation above depends on having a certain amount of time in each
        # loop.
        if fall_timer.time() > 1000:
            break
        wait(TARGET_LOOP_PERIOD - single_loop_timer.time())
        
    print(' Gyro Offset: {} \n Gyro Angle: {} '.format( gyro_offset, gyro_sensor.angle()))
    
    last_error_angle = angle -  gyro_sensor.angle()
    while last_error_angle != 0 :
        error_angle = angle -  gyro_sensor.angle()
        # wait(3000)
        print('error_angle', error_angle)
        integral = integral + error_angle
        derivative = last_error_angle

        turn_rate = PROPORTIONAL_GAIN * last_error_angle 
        robot.drive(0, turn_rate)
        wait(300)
        error_angle = angle -  gyro_sensor.angle()
        # wait(3000)
        last_error_angle = error_angle
        wait(TARGET_LOOP_PERIOD - single_loop_timer.time())
    gyro_sensor.reset_angle(0)


gyro_sensor.reset_angle(0)

# while True:
    # Reset the sensors and variables.
    

    # print(a_angle())
    # # Calibrate the gyro offset. This makes sure that the robot is perfectly
    # # still by making sure that the measured rate does not fluctuate more than
    # # 2 deg/s. Gyro drift can cause the rate to be non-zero even when the robot
    # # is not moving, so we save that value for use later.
    # while True:
    #     gyro_minimum_rate, gyro_maximum_rate = 440, -440
    #     gyro_sum = 0
    #     for _ in range(GYRO_CALIBRATION_LOOP_COUNT):
    #         gyro_sensor_value = gyro_sensor.angle()
    #         gyro_sum += gyro_sensor_value
    #         if gyro_sensor_value > gyro_maximum_rate:
    #             gyro_maximum_rate = gyro_sensor_value
    #         if gyro_sensor_value < gyro_minimum_rate:
    #             gyro_minimum_rate = gyro_sensor_value
    #         wait(5)
    #     print('Gyro Rate: {} '.format(gyro_maximum_rate -
    #           gyro_minimum_rate, gyro_sensor.angle()))
    #     if gyro_maximum_rate - gyro_minimum_rate < 2:
    #         break
    # gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT

    # while True:
    #     # calculate robot body angle and speed
    #     gyro_sensor_value = gyro_sensor.angle()
    #     gyro_offset *= (1 - GYRO_OFFSET_FACTOR)
    #     gyro_offset += GYRO_OFFSET_FACTOR * gyro_sensor_value
    #     robot_body_rate = gyro_sensor_value - gyro_offset
    #     # Make sure loop time is at least TARGET_LOOP_PERIOD. The output power
    #     # calculation above depends on having a certain amount of time in each
    #     # loop.
    #     if fall_timer.time() > 1000:
    #         break

    #     wait(TARGET_LOOP_PERIOD - single_loop_timer.time())
    # print(' Gyro Offset: {} \n Gyro Angle: {} \nrobot_body_rate :{}'.format(
    #     gyro_offset, gyro_sensor.angle(), robot_body_rate))
    # print(a_angle())
robot_turn_PID(90)
wait(500)
robot.straight(300)
wait(500)
robot_turn_PID(90)
wait(500)
robot.straight(300)
wait(500)
robot_turn_PID(90)
wait(500)

    # wait(3000)
