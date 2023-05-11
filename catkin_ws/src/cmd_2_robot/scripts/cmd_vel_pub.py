import rospy
from geometry_msgs.msg import Twist

def cmd_talker():
    rospy.init_node('cmd_talker', anonymous=True)
    pub = rospy.Publisher('run_cmd', Twist, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0
        rospy.loginfo("x = {}, y = {}, z = {}".format(twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z))

        pub.publish(twist_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        cmd_talker()
    except rospy.ROSInterruptException:
        pass