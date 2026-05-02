import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

def main():
    rospy.init_node('usb_camera_publisher', anonymous=True)

    image_pub = rospy.Publisher('/usb_camera/image', Image, queue_size=10)
    compressed_image_pub = rospy.Publisher('/usb_camera/image/compressed', CompressedImage, queue_size=10)

    bridge = CvBridge()

    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
  
    while not cap.isOpened() and not rospy.is_shutdown():
        rospy.logwarn("Waiting for USB camera to be available...")
        rospy.sleep(1)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture image from USB camera.")
            continue

        try:
            timestamp = rospy.Time.now()

            image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_msg.header.stamp = timestamp

            image_pub.publish(image_msg)

            compressed_image_msg = CompressedImage()
            compressed_image_msg.header.stamp = timestamp
            compressed_image_msg.format = "jpeg"
            compressed_image_msg.data = cv2.imencode('.jpg', frame)[1].tobytes()
            compressed_image_pub.publish(compressed_image_msg)

        except Exception as e:
            rospy.logerr("Error processing frame: %s" % str(e))

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass