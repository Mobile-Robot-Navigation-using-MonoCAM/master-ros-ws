#! /usr/bin/python3
import rospy
from geometry_msgs.msg import Twist

pub = None

def main():
    
    global pub
    
    rospy.init_node('stop')
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # i = 0
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # i += 1
        if pub.get_num_connections() >= 1:
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            pub.publish(twist_msg)
            break
        # print("No. of steps: ", i)
        rate.sleep()

if __name__ == "__main__":
    main()
