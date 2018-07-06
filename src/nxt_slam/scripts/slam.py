#!/usr/bin/env python
# From: https://github.com/Eelviny/nxt-python/blob/master/nxt/motor.py

import nxt.locator
import time
import nxt.bluesock

from nxt.sensor import *
from nxt.motor import Motor, PORT_A, PORT_B, PORT_C

brick = nxt.bluesock.BlueSock('00:16:53:04:17:F1').connect()

motor_right = Motor(brick, PORT_A)
motor_left = Motor(brick, PORT_C)
ultrasonic = Ultrasonic(brick, PORT_2)


def CommunicateWithRobot():
    while(True):
        print('Motor A Tacho: ' + str(motor_right.get_tacho()))
        print('Motor C Tacho: ' + str(motor_left.get_tacho()))
        print('Ultrasonic: ' + str(Ultrasonic(brick, PORT_2).get_sample()))
        time.sleep(0.2)


if __name__ == '__main__':
    CommunicateWithRobot()
