#! /usr/bin/python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

def image_callback(ros_image):
    # Convert ROS image message to OpenCV image
    bridge = CvBridge()
    
    try:
        # Convert your ROS Image message to ROS CompressImage
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        compress_img = bridge.cv2_to_compressed_imgmsg(cv_image, "jpeg")
        pub.publish(compress_img)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':

    pub_topic = "/camera/image_raw/compressedimage"
    sub_topic = "/camera/image_raw"

    try:
        rospy.init_node('process_image', anonymous=True)
        pub = rospy.Publisher(pub_topic, CompressedImage, queue_size=10)
        rospy.Subscriber(sub_topic, Image, image_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass