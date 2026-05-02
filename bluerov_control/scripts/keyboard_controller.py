#!/usr/bin/env python
import math
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
from bluerov_bridge import Bridge

x, y, z, yaw = 0, 0, 500, 0

if_arm = False
if_depth_hold = False

last_if_arm = False
last_if_depth_hold = False

if_go_circle = False

cof = 0.4

def handle_key(key):
    global x, y, z, yaw, if_arm, if_depth_hold, if_go_circle, cof

    if key == '1':
        if_arm = True
        return
    
    if key == '2':
        if_arm = False
        return

    if key == '3':
        if_depth_hold = True
        return
    
    if key == '4':
        if_depth_hold = False
        return

    if key == '5':
        if_go_circle = True
    
    if key == '6':
        if_go_circle = False

    if key == '7':
        cof += 0.02
    if key == '8':
        cof -= 0.02

    if key == 'w':
        if x < 0:
            x = 0
        elif x >= 1000:
            x = 1000
        else:
            x += 100
        return
        
    if key == 's':
        if x > 0:
            x = 0
        elif x <= -1000:
            x = -1000
        else:
            x -= 100
        return
        
    if key == 'a':
        if y > 0:
            y = 0
        elif y <= -1000:
            y = -1000
        else:
            y -= 100
        return

    if key == 'd':
        if y < 0:
            y = 0
        elif y >= 1000:
            y = 1000
        else:
            y += 100
        return
        
    if key == 'u':
        if z < 500:
            z = 500
        elif z >= 1000:
            z = 1000
        else:
            z += 50
        return

    if key == 'j':
        if z > 500:
            z = 500
        elif z <= 0:
            z = 0
        else:
            z -= 50
        return
    
    if key == 'i':
        if yaw < 0:
            yaw = 0
        elif yaw >= 1000:
            yaw = 1000
        else:
            yaw += 100
        return
    
    if key == 'k':
        if yaw > 0:
            yaw = 0
        elif yaw <= -1000:
            yaw = -1000
        else:
            yaw -= 100
        return

if __name__ == '__main__':
    print('welcome to bluerov control..')
    rospy.init_node('keyboard_control')
    
    device = 'udp:192.168.2.1:14550'
    while not rospy.is_shutdown():
        try:
            bridge = Bridge(device)
        except socket.error:
            rospy.logerr('Failed to make mavlink connection to device {}'.format(device))
        else:
            break
    if rospy.is_shutdown():
        sys.exit(-1)

    print("connect rov success..")
    
    rate = rospy.Rate(5)

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    bridge.calibrate_pressure()

    while not rospy.is_shutdown():
        bridge.update()

        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)

            if key == '\x1b':
                break
            
            handle_key(key)

        print("x: ", x, "  y: ", y, "  z: ", z, "  yaw: ", yaw, "  if_arm: ", if_arm, "  if_depth_hold: ", if_depth_hold, "  if_go_circle: ", if_go_circle, "  cof: ", cof)
        
        if last_if_arm != if_arm:
            if if_arm:  
                bridge.arm()
            else:
                bridge.disarm()

        if last_if_depth_hold != if_depth_hold:
            if if_depth_hold:
                bridge.set_mode('alt_hold')
            else:
                bridge.set_mode('manual')
        
        if if_go_circle == True:
            bridge.go_circlre(0.5, 500, False, cof)
        else:
            bridge.manual_control(x, y, z, yaw)

        last_if_arm = if_arm
        last_if_depth_hold = if_depth_hold

        rate.sleep()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    bridge.disarm()
    bridge.set_mode('manual')