#!/usr/bin/env python
'''
Copyright (C), CASIA 2022.
File name: 
Author: Ou Yaming(ouyaming2021@ia.ac.cn)
Version: V1.0
Date: 2022-11-02 10:18:06
Description: 
Others: None
History: <time>   <author>    <version >   <desc>
'''

import rospy
from sensor_msgs.msg import Imu
from wldvl import WlDVL
from time import sleep


rospy.init_node("dvl_pub")

dvl_velocity_pub = rospy.Publisher("dvl_velocity",Imu,queue_size=1)
dvl_position_pub = rospy.Publisher("dvl_position",Imu,queue_size=1)

rate = rospy.Rate(500)

dvl = WlDVL("/dev/ttyUSB3")
print("connet dvl success..")
print("dead reckoning reset..")
dvl.position_reset()
dvl.position_reset()
dvl.position_reset()
sleep(1)

print("start read dvl data..")
while not rospy.is_shutdown():
    velocity_imumsg = Imu()
    position_imumsg = Imu()

    dvl_msg = dvl.read()
    if dvl_msg:
        print(dvl_msg)
        if dvl_msg['property'] == 'velocity' and dvl_msg['valid']:
            velocity_imumsg.header.frame_id = "dvl_link"
            velocity_imumsg.header.stamp = rospy.Time.from_sec(dvl_msg['time_stamp'])
            velocity_imumsg.orientation.z = dvl_msg["altitude"]
            velocity_imumsg.linear_acceleration.x = dvl_msg["vx"]
            velocity_imumsg.linear_acceleration.y = dvl_msg["vy"]
            velocity_imumsg.linear_acceleration.z = dvl_msg["vz"]
            velocity_imumsg.orientation.w = dvl_msg["fom"]

            dvl_velocity_pub.publish(velocity_imumsg)


        elif dvl_msg['property'] == 'position' and dvl_msg['status'] < 1:
            position_imumsg.header.frame_id = "dvl_link"
            position_imumsg.header.stamp = rospy.Time.from_sec(dvl_msg['time_stamp'])
            position_imumsg.orientation.x = dvl_msg['x']
            position_imumsg.orientation.y = dvl_msg['y']
            position_imumsg.orientation.z = dvl_msg['z']
            position_imumsg.orientation.w = dvl_msg['pos_std']
            position_imumsg.angular_velocity.x = dvl_msg['roll']
            position_imumsg.angular_velocity.y = dvl_msg['pitch']
            position_imumsg.angular_velocity.z = dvl_msg['yaw']

            dvl_position_pub.publish(position_imumsg)
    

    rate.sleep()
