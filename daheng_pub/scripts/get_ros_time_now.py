import rospy

rospy.init_node('time_test_node')

rate = rospy.Rate(10)  # 设置发布频率为 10Hz
while not rospy.is_shutdown():
    rospy.loginfo("Laptop ROS time: %s", rospy.get_time())
    rate.sleep()  # 按照设定频率发布