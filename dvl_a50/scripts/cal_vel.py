# !/usr/bin/env python
'''
Copyright (C), CASIA 2022.
File name: 
Author: Ou Yaming(ouyaming2021@ia.ac.cn)
Version: V1.0
Date: 2022-09-26 14:15:07
Description: 
Others: None
History: <time>   <author>    <version >   <desc>
'''
import math
import numpy as np
from socket import socket
from pymavlink import mavutil
import rospy
import sys
import socket
import tty
import termios
import os
import threading
import select
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import time




current_vx = 0
current_vy = 0
current_vz = 0

current_velocity = 0

sum_velocity = 0
sum_number = 0

max_velocity = 0


def _callback_dvl_velocity(msg):
    global current_vx,current_vy,current_vz,current_velocity,sum_velocity,sum_number,max_velocity
    current_vx = msg.linear_acceleration.x
    current_vy = msg.linear_acceleration.y
    current_vz = msg.linear_acceleration.z

    current_velocity = math.sqrt(current_vx ** 2 + current_vy ** 2 + current_vz ** 2)

    if current_velocity > max_velocity:
        max_velocity = current_velocity

    sum_velocity += current_velocity
    sum_number += 1


if __name__ == '__main__':
    print('welcome to bluerov control..')
    rospy.init_node('oym_commander')

    rospy.Subscriber('dvl_velocity', Imu, _callback_dvl_velocity)

    while not rospy.is_shutdown():
        if sum_number:
            ave_velocity = sum_velocity/sum_number
            print("ave_velocity: ",ave_velocity,"  max_velocity: ",max_velocity)
