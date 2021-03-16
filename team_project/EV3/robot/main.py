#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.iodevices import UARTDevice
import time
# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
front_sensor = UltrasonicSensor(Port.S1) 
MOTOR_SPD = 100
CAM_WIDTH = 800
# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=120)
ev3.speaker.beep()
uart = UARTDevice(Port.S4, baudrate=9600, timeout=None)
be_person = False
waiting_robot = False

def turn_door():
    max_dist_value = 0
    max_dist_angle = 0
    robot.reset()
    # 한바귀 돌아서 거리값 측정하며 최대 거리값과 그값의 각도 구하기
    while True:
        left_motor.run(MOTOR_SPD)
        right_motor.run(-MOTOR_SPD)
        # print("ROBOT || ANGLE : {} || DISTANCE : {}".format(robot.angle(), front_sensor.distance()))

        if max_dist_value < front_sensor.distance():
            max_dist_value = front_sensor.distance()
            max_dist_angle = robot.angle()

        if robot.angle() >= 360:
            left_motor.brake()
            right_motor.brake()
            break

    wait(200)
    robot.reset()
    rotation_dir = robot.angle() - max_dist_angle + 180

    if rotation_dir < 0: # 반시계방향 회전
        while True:
            left_motor.run(-MOTOR_SPD/2)
            right_motor.run(MOTOR_SPD/2)
            if robot.angle() == max_dist_angle-360:
                break
            elif robot.angle() < -360:
                break

    elif rotation_dir >= 0: # 시계방향 회전
        while True:
            left_motor.run(MOTOR_SPD/2)
            right_motor.run(-MOTOR_SPD/2)
            if robot.angle() == max_dist_angle:
                break
            elif robot.angle() > 360:
                break
    wait(500)


def get_str():
    dat  = "";  cnt = 0;    rcv = []
    
    while True:
        rxi = uart.read(1)
        
        if rxi != b'\n' and cnt < 19:
            cnt += 1;   rcv.append(rxi.decode())
        else:
            dat = "".join(rcv)
            return dat


person_in_room = False
high_temp = False
# 문을 향하서 회전한다.
# turn_door()
# 받은 사람의 중앙좌표값으로 사람을 향하도록 회전한다.
waiting_robot = True
while True:
    # 사람인식
    cmd = get_str()
    # print(cmd)
    if person_in_room == False:
        if cmd !='-1' and cmd != '1':
            try:
                cmd = int(cmd)
                person_in_room = True
                # print("Person Here!")
            except:
                pass
                
    # print(person_in_room)
    # 온도여부확인
    if cmd == "FON":
        high_temp = True
    elif cmd == "FOF":
        high_temp = False
    # 온도가 높아져 릴레이모듈 켜지면 회전사람 찾는다.
    if high_temp == True and person_in_room == True:
        print("Run Robot")
        waiting_robot = False
        while True:
            # 시계방향 회전한다.
            left_motor.run(MOTOR_SPD/2)
            right_motor.run(-MOTOR_SPD/2)
            cmd = get_str()
            # search_person
            # 사람 좌표값이 받아지면 멈춘다
            if cmd !='-1' and cmd != '1':
                try:
                    cmd = int(cmd)
                    left_motor.brake()
                    right_motor.brake()
                    break
                except:
                    pass
            
            if robot.angle() > 400:
                person_in_room = False
                left_motor.brake()
                right_motor.brake()
                break

    # toward_person
        threshold = 5
        imC = int(CAM_WIDTH/2)
        imC_offset = 150
        not_detect_cnt = 0
        detected_person = False
        while True:
            cmd = get_str()
            try:
                cmd = int(cmd)
                if cmd == -1:
                    left_motor.brake()
                    right_motor.brake()
                    detected_person = False
                    not_detect_cnt += 1
                    print("Person Not Detect")
                else:
                    detected_person = True
                    not_detect_cnt = 0
                    if cmd < imC - imC_offset and cmd >= 0:
                        left_motor.run(-MOTOR_SPD/2)
                        right_motor.run(MOTOR_SPD/2)
                        # print("Turn Left")
                    elif cmd > imC + imC_offset and cmd <= CAM_WIDTH:
                        left_motor.run(MOTOR_SPD/2)
                        right_motor.run(-MOTOR_SPD/2)
                        # print("Turn Right")
                    else:
                        # print("Center")
                        if front_sensor.distance > 700:
                            left_motor.run(MOTOR_SPD/2)
                            right_motor.run(MOTOR_SPD/2)
                        elif front_sensor.distance < 400:
                            left_motor.run(-MOTOR_SPD/2)
                            right_motor.run(-MOTOR_SPD/2)
                        else:
                            left_motor.brake()
                            right_motor.brake()


                if not_detect_cnt > threshold:
                    # print("person leave room")
                    break

                if cmd == "FOF":
                    high_temp = False
                    left_motor.brake()
                    right_motor.brake()
                    break
            except:
                if cmd == "FOF":
                    high_temp = False
                    left_motor.brake()
                    right_motor.brake()
                    break


    if waiting_robot == False and person_in_room == False:
        # print("search door")
        turn_door()
        waiting_robot = True
        wait(500)
        left_motor.brake()
        right_motor.brake()

    if waiting_robot == False and high_temp == False:
        # print("search door")
        turn_door()
        waiting_robot = True
        wait(500)
        left_motor.brake()
        right_motor.brake()


