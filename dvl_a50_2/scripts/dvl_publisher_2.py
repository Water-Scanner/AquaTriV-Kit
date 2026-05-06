#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import sys
import os
import tf
from time import sleep
sys.path.append(os.path.join(os.path.dirname(__file__)))
from wldvl import WlDVL
from dvl_a50_2.msg import DVL
from dvl_a50_2.msg import DVLBeam
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from udp_float_sender import create_float_ascii_sender

# UDP configuration for altitude sending
IP = "192.168.2.1"
PORT = 5000
PRECISION = 3
TOPIC = "/dvl/data"

# Utility function: Convert dictionary to DVL message
def dict_to_dvl_msg(data):
    msg = DVL()
    # Header
    msg.header = Header()
    if 'header' in data and 'stamp' in data['header']:
        msg.header.stamp.secs = data['header']['stamp'].get('secs', 0)
        msg.header.stamp.nsecs = data['header']['stamp'].get('nsecs', 0)
        msg.header.frame_id = data['header'].get('frame_id', '')
    # Scalar fields
    msg.time = data.get('time', 0.0)
    msg.fom = data.get('fom', 0.0)
    msg.altitude = data.get('altitude', 0.0)
    msg.velocity_valid = data.get('velocity_valid', False)
    # Velocity
    msg.velocity = Vector3()
    if 'velocity' in data:
        msg.velocity.x = data['velocity'].get('x', 0.0)
        msg.velocity.y = data['velocity'].get('y', 0.0)
        msg.velocity.z = data['velocity'].get('z', 0.0)
    # Beams
    msg.beams = []
    for beam_dict in data.get('beams', []):
        beam = DVLBeam()
        beam.id = int(beam_dict.get('id', 0))
        beam.velocity = beam_dict.get('velocity', 0.0)
        beam.distance = beam_dict.get('distance', 0.0)
        beam.rssi = beam_dict.get('rssi', 0.0)
        beam.nsd = beam_dict.get('nsd', 0.0)
        beam.valid = beam_dict.get('valid', False)
        msg.beams.append(beam)
    return msg

class DVLPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('dvl_publisher', anonymous=True)

        # Get parameters
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.debug = rospy.get_param('~debug', False)

        # Create publishers
        self.velocity_pub = rospy.Publisher('/dvl/velocity', Imu, queue_size=10)
        self.position_pub = rospy.Publisher('/dvl/position', Imu, queue_size=10)
        self.dvl_pub = rospy.Publisher('/dvl/data', DVL, queue_size=10)

        # Initialize TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Initialize UDP altitude sender (reusable socket)
        self._altitude_sender = create_float_ascii_sender(IP, PORT, precision=PRECISION)

        # Try to connect to DVL device
        try:
            self.dvl = WlDVL(self.port, baudrate=self.baud_rate, debug=self.debug)
            print("Connected to DVL successfully...")
            print("Resetting dead reckoning...")
            self.dvl.position_reset()
            self.dvl.position_reset()
            self.dvl.position_reset()
            self.dvl.position_reset()
            sleep(2)
            rospy.loginfo("Connected to DVL device {} at baud rate {}.".format(self.port, self.baud_rate))
        except Exception as e:
            rospy.logerr("Unable to open DVL device {}: {}".format(self.port, e))
            rospy.signal_shutdown("Failed to open DVL device")

        # Set refresh rate
        self.rate = rospy.Rate(400)  # 100 Hz (adjusted for typical DVL rates)

    def publish_data(self):
        while not rospy.is_shutdown():
            try:
                data = self.dvl.read()
                if data:  # Publish only when complete data is received
                    if data.get('property') == 'velocity':
                        dvl_msg = dict_to_dvl_msg(data)
                        self.dvl_pub.publish(dvl_msg)
                        self.publish_velocity_data(data)
                    elif data.get('property') == 'position':
                        self.publish_position_data(data)
            except Exception as e:
                rospy.logerr(f"Error reading or publishing DVL data: {e}")
            self.rate.sleep()

    def publish_position_data(self, packet):
        # Create Imu message
        position_imumsg = Imu()
        position_imumsg.header.frame_id = "dvl_link"
        position_imumsg.header.stamp = rospy.Time.now()
        position_imumsg.orientation.x = packet.get('x', 0.0)
        position_imumsg.orientation.y = packet.get('y', 0.0)
        position_imumsg.orientation.z = packet.get('z', 0.0)
        position_imumsg.orientation.w = packet.get('pos_std', 0.0)
        position_imumsg.angular_velocity.x = packet.get('roll', 0.0)
        position_imumsg.angular_velocity.y = packet.get('pitch', 0.0)
        position_imumsg.angular_velocity.z = packet.get('yaw', 0.0)
        self.position_pub.publish(position_imumsg)
        rospy.logdebug("Published position: x={:.2f}, y={:.2f}, z={:.2f}, pos_std={:.2f}".format(
            packet.get('x', 0.0), packet.get('y', 0.0), packet.get('z', 0.0), packet.get('pos_std', 0.0)))

    def publish_velocity_data(self, packet):
        # Create Imu message
        velocity_imumsg = Imu()
        velocity_imumsg.header.frame_id = "dvl_link"
        velocity_imumsg.header.stamp = rospy.Time.now()
        velocity_imumsg.orientation.z = packet.get("altitude", 0.0)
        velocity_imumsg.linear_acceleration.x = packet['velocity'].get('x', 0.0)
        velocity_imumsg.linear_acceleration.y = packet['velocity'].get('y', 0.0)
        velocity_imumsg.linear_acceleration.z = packet['velocity'].get('z', 0.0)
        velocity_imumsg.orientation.w = packet.get("fom", 0.0)
        self.velocity_pub.publish(velocity_imumsg)

        # Send altitude over UDP as ASCII with configured precision
        try:
            altitude = float(packet.get("altitude", 0.0))
            self._altitude_sender(altitude)
        except Exception as e:
            rospy.logwarn(f"Failed to send altitude over UDP: {e}")

if __name__ == '__main__':
    try:
        dvl_publisher = DVLPublisher()
        dvl_publisher.publish_data()
    except rospy.ROSInterruptException:
        pass