#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np

class MergedImagePublisher:
    def __init__(self, publish_rate):
        rospy.init_node('merged_image_publisher_node', anonymous=True)

        self.left_image_sub = Subscriber('/binocular/left_image', Image)
        self.right_image_sub = Subscriber('/binocular/right_image', Image)

        # Using ApproximateTimeSynchronizer for time synchronization
        self.sync = ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub], queue_size=10, slop=0.005)
        self.sync.registerCallback(self.image_callback)

        self.bridge = CvBridge()

        self.publish_rate = publish_rate  # Publishing rate in Hz

        # ROS publisher for the merged image
        self.merged_image_pub = rospy.Publisher('/binocular/merged_image', Image, queue_size=10)

        self.last_publish_time = rospy.get_time()  # Initialize the last publish time

    def image_callback(self, left_msg, right_msg):
        try:
            # Convert ROS Image messages to OpenCV images
            left_image = self.bridge.imgmsg_to_cv2(left_msg, "8UC1")
            right_image = self.bridge.imgmsg_to_cv2(right_msg, "8UC1")
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert image message to OpenCV format: {e}")
            return

        # Get the current time
        current_time = rospy.get_time()

        # Check if enough time has passed to publish the images
        if (current_time - self.last_publish_time) >= (1.0 / self.publish_rate):
            try:
                # Merge left and right images side by side
                merged_image = np.hstack((left_image, right_image))

                # Convert the merged OpenCV image back to a ROS Image message
                merged_image_msg = self.bridge.cv2_to_imgmsg(merged_image, encoding="mono8")

                # Publish the merged image
                self.merged_image_pub.publish(merged_image_msg)

                rospy.loginfo("Published merged image.")

                # Update the last publish time
                self.last_publish_time = current_time

            except CvBridgeError as e:
                rospy.logerr(f"Failed to convert OpenCV image to ROS message: {e}")

if __name__ == '__main__':
    publish_rate = 5  # Publishing rate in Hz
    merged_image_publisher = MergedImagePublisher(publish_rate)
    rospy.spin()
