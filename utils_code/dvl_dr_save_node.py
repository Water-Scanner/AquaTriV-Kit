#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
import os

class DVLToTUM:
    def __init__(self):
        rospy.init_node('dvl_to_tum_logger', anonymous=True)

        self.output_path = rospy.get_param('~output_path', 'dvl_trajectory_tum.txt')
        self.output_path = os.path.abspath(self.output_path)
        
        rospy.loginfo(f"Data will be saved to: {self.output_path}")

        with open(self.output_path, 'w') as f:
            f.write("# TUM format: timestamp x y z qx qy qz qw\n")

        self.sub = rospy.Subscriber('/dvl/position', Odometry, self.odom_callback)
        
        self.count = 0

    def odom_callback(self, msg):
        try:
            timestamp = msg.header.stamp.to_sec()
            
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            tum_line = f"{timestamp:.4f} {x:.4f} {y:.4f} {z:.4f} {qx:.4f} {qy:.4f} {qz:.4f} {qw:.4f}\n"

            with open(self.output_path, 'a') as f:
                f.write(tum_line)

            self.count += 1
            if self.count % 10 == 0:
                rospy.loginfo(f"{self.count} trajectory records written...")

        except Exception as e:
            rospy.logerr(f"Failed to write TUM data: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = DVLToTUM()
        logger.run()
    except rospy.ROSInterruptException:
        pass