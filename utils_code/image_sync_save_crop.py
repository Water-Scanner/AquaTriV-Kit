#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from message_filters import ApproximateTimeSynchronizer, Subscriber

class ImageSaver:
    def __init__(self, save_rate, main_folder_path):
        rospy.init_node('image_saver_node', anonymous=True)

        self.left_image_sub = Subscriber('/topic_1', Image)
        self.right_image_sub = Subscriber('/topic_2', Image)

        self.sync = ApproximateTimeSynchronizer([self.left_image_sub, self.right_image_sub], queue_size=10, slop=0.01)
        self.sync.registerCallback(self.image_callback)

        self.bridge = CvBridge()

        self.save_rate = save_rate
        self.counter = 0

        self.main_folder_path = main_folder_path
        self.save_path_left = os.path.join(self.main_folder_path, 'left')
        self.save_path_right = os.path.join(self.main_folder_path, 'right')

        if not os.path.exists(self.save_path_left):
            os.makedirs(self.save_path_left)

        if not os.path.exists(self.save_path_right):
            os.makedirs(self.save_path_right)

        self.last_save_time = rospy.get_time()

    def image_callback(self, left_msg, right_msg):
        try:
            left_image = self.bridge.imgmsg_to_cv2(left_msg, "8UC1")
            right_image_color = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")
            right_image = cv2.cvtColor(right_image_color, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            rospy.logerr(e)
            return

        current_time = rospy.get_time()
        if (current_time - self.last_save_time) >= (1.0 / self.save_rate):
            left_image_filename = f"{self.counter}.png"
            right_image_filename = f"{self.counter}.png"

            target_width = 808
            target_height = 608
            height, width = right_image.shape[:2]

            width_ratio = target_width / float(width)
            height_ratio = target_height / float(height)

            ratio = max(width_ratio, height_ratio)

            print("width_ratio: ", width_ratio, " height_ratio: ", height_ratio, " ratio: ", ratio)

            new_width = int(width * ratio)
            new_height = int(height * ratio)

            print("new_width: ", new_width, " new_height: ", new_height)

            right_image = cv2.resize(right_image, (new_width, new_height), interpolation=cv2.INTER_AREA)

            height, width = right_image.shape[:2]

            x_center = width // 2
            y_center = height // 2
            crop_width = 808
            crop_height = 608

            x_start = max(0, x_center - crop_width // 2)
            y_start = max(0, y_center - crop_height // 2)

            right_image = right_image[y_start:y_start + crop_height, x_start:x_start + crop_width]

            height, width = right_image.shape[:2]
            print("after height: ", height, " width: ", width)

            cv2.imwrite(os.path.join(self.save_path_left, left_image_filename), left_image)
            cv2.imwrite(os.path.join(self.save_path_right, right_image_filename), right_image)

            rospy.loginfo(f"Image pair saved: {os.path.join(self.save_path_left, left_image_filename)}, {os.path.join(self.save_path_right, right_image_filename)}")

            self.last_save_time = current_time
            self.counter += 1

if __name__ == '__main__':
    save_rate = 3
    main_folder_path = "path"
    image_saver = ImageSaver(save_rate, main_folder_path)
    rospy.spin()