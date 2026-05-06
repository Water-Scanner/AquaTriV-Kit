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
from dvl_a50.msg import DVL
from dvl_a50.msg import DVLBeam
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

# 工具函数：字典转DVL消息

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
    msg.status = int(data.get('status', 0))
    msg.form = data.get('form', '')
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
        # 初始化 ROS 节点
        rospy.init_node('dvl_publisher', anonymous=True)

        # 获取参数
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.debug = rospy.get_param('~debug', False)

        # 创建发布者
        self.velocity_pub = rospy.Publisher('/dvl/velocity', Imu, queue_size=10)
        self.position_pub = rospy.Publisher('/dvl/position', Imu, queue_size=10)
        self.dvl_pub = rospy.Publisher('/dvl/data', DVL, queue_size=10)


        # 初始化 tf 广播器
        self.tf_broadcaster = tf.TransformBroadcaster()

        # 尝试连接 DVL 设备
        try:
            self.dvl = WlDVL(self.port, baudrate=self.baud_rate, debug=self.debug)
            print("连接 DVL 成功...")
            print("航位推算重置...")
            self.dvl.position_reset()
            self.dvl.position_reset()
            self.dvl.position_reset()
            self.dvl.position_reset()
            sleep(2)
            rospy.loginfo("已连接到 DVL 设备 {}，波特率 {}。".format(self.port, self.baud_rate))
        except Exception as e:
            rospy.logerr("无法打开 DVL 设备 {}: {}".format(self.port, e))
            rospy.signal_shutdown("DVL 设备打开失败")

        # 设置刷新频率
        self.rate = rospy.Rate(200)  # 100 Hz

    def publish_data(self):
        while not rospy.is_shutdown():
            try:
                data = self.dvl.read()
                if data:  # 只在收到完整一组数据时才发布
                    dvl_msg = dict_to_dvl_msg(data)
                    self.dvl_pub.publish(dvl_msg)
                    # 发布原始航迹推算位姿
                    if 'position' in data:
                        self.publish_position_data(data['position'])
                    # 发布原始速度信息（如果velocity字段为wrx原始包）
                    if 'velocity' in data and all(k in data['velocity'] for k in ['x','y','z']):
                        # 为速度数据添加时间戳字段
                        velocity_data = data['velocity'].copy()
                        velocity_data['time_stamp'] = data.get('time', rospy.Time.now().to_sec())
                        self.publish_velocity_data(velocity_data)
            except Exception as e:
                rospy.logerr(f"Error reading or publishing DVL data: {e}")
            self.rate.sleep()

    def publish_dvl_message(self, data):
        """Publish the DVL message according to the specified format"""
        msg = DVL()
        
        # Set header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        
        # Set time (convert from parser's time to ROS time if needed)
        msg.time = data['time'] if 'time' in data else 0.0
        
        # Set velocity
        velocity = Vector3()
        velocity.x = data['velocity']['x']
        velocity.y = data['velocity']['y']
        velocity.z = data['velocity']['z']
        msg.velocity = velocity
        
        # Set other fields
        msg.fom = data.get('fom', 0.0)
        msg.altitude = data.get('altitude', 0.0)
        msg.velocity_valid = data.get('velocity_valid', False)
        msg.status = data.get('status', 0)
        msg.form = data.get('form', '')
        
        # Set beam data
        for beam_data in data['beams']:
            beam = DVLBeam()
            beam.id = beam_data['id']
            beam.velocity = beam_data['velocity']
            beam.distance = beam_data['distance']
            beam.rssi = beam_data.get('rssi', 0.0)
            beam.nsd = beam_data.get('nsd', 0.0)
            beam.valid = beam_data.get('valid', False)
            msg.beams.append(beam)
            
        self.dvl_pub.publish(msg)

    def publish_position_data(self, packet):
        # 发布位置数据为 Imu 消息格式
        position_imumsg = Imu()
        position_imumsg.header.frame_id = "dvl_link"
        position_imumsg.header.stamp = rospy.Time.from_sec(packet.get('time_stamp', rospy.Time.now().to_sec()))
        position_imumsg.orientation.x = packet["x"]
        position_imumsg.orientation.y = packet["y"]
        position_imumsg.orientation.z = packet["z"]
        position_imumsg.orientation.w = packet.get("pos_std", 0.0)
        position_imumsg.angular_velocity.x = packet["roll"]
        position_imumsg.angular_velocity.y = packet["pitch"]
        position_imumsg.angular_velocity.z = packet["yaw"]

        self.position_pub.publish(position_imumsg)
        rospy.logdebug("发布位置: x={:.2f}, y={:.2f}, z={:.2f}".format(
            packet["x"], packet["y"], packet["z"]))

        # 广播 dvl_frame 到 dvl_map_frame 的变换
        import math
        quaternion = tf.transformations.quaternion_from_euler(
            packet["roll"] * math.pi / 180.0,  # roll (X轴)
            packet["pitch"] * math.pi / 180.0,  # pitch (Y轴)
            packet["yaw"] * math.pi / 180.0,    # yaw (Z轴)
            'rxyz'
        )
        self.tf_broadcaster.sendTransform(
            (packet["x"], packet["y"], packet["z"]),  # 平移
            quaternion,  # 旋转
            rospy.Time.now(),
            "dvl_frame",  # 子坐标系
            "dvl_map_frame"  # 父坐标系
        )

    def publish_velocity_data(self, packet):
        # 发布速度数据为 Imu 消息格式
        velocity_imumsg = Imu()
        velocity_imumsg.header.frame_id = "dvl_link"
        velocity_imumsg.header.stamp = rospy.Time.from_sec(packet.get('time_stamp', rospy.Time.now().to_sec()))
        velocity_imumsg.orientation.z = packet.get("altitude", 0.0)
        velocity_imumsg.linear_acceleration.x = packet["x"]
        velocity_imumsg.linear_acceleration.y = packet["y"]
        velocity_imumsg.linear_acceleration.z = packet["z"]
        velocity_imumsg.orientation.w = packet.get("fom", 0.0)

        self.velocity_pub.publish(velocity_imumsg)
        rospy.logdebug("发布速度: x={:.2f}, y={:.2f}, z={:.2f}, 时间戳: {}".format(
            velocity_imumsg.linear_acceleration.x,
            velocity_imumsg.linear_acceleration.y,
            velocity_imumsg.linear_acceleration.z,
            velocity_imumsg.header.stamp))

if __name__ == '__main__':
    try:
        dvl_publisher = DVLPublisher()
        dvl_publisher.publish_data()
    except rospy.ROSInterruptException:
        pass
