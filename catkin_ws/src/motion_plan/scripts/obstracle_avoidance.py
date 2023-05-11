#! /usr/bin/python3
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = None

def clbk_laser(msg):
    regions = {
        "right": min(min(msg.ranges[0:143]), 10),
        "fright": min(min(msg.ranges[144:286]), 10),
        "front": min(min(msg.ranges[287:430]), 10),
        "fleft": min(min(msg.ranges[431:574]), 10),
        "left": min(min(msg.ranges[575:719]), 10)

    }

    
    take_action(regions)
    # rospy.loginfo(regions)

def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_discription = ""

    
    if regions["front"] > 1 and regions["fleft"] > 1 and regions["fright"] > 1:
        state_discription = "case-1 none"
        linear_x = 0.6
        angular_z = 0
    elif regions["front"] < 1 and regions["fleft"] > 1 and regions["fright"] > 1:
        state_discription = "case-2 front"
        linear_x = 0
        angular_z = -0.3
    elif regions["front"] > 1 and regions["fleft"] > 1 and regions["fright"] < 1:
        state_discription = "case-3 front_right"
        linear_x = 0
        angular_z = -0.3
    elif regions["front"] > 1 and regions["fleft"] < 1 and regions["fright"] > 1:
        state_discription = "case-4 front_left"
        linear_x = 0
        angular_z = 0.3
    elif regions["front"] < 1 and regions["fleft"] > 1 and regions["fright"] < 1:
        state_discription = "case-5 front and fright"
        linear_x = 0
        angular_z = -0.3
    elif regions["front"] < 1 and regions["fleft"] < 1 and regions["fright"] > 1:
        state_discription = "case-6 front and fleft"
        linear_x = 0
        angular_z = 0.3
    elif regions["front"] < 1 and regions["fleft"] < 1 and regions["fright"] < 1:
        state_discription = "case-7 front and fright and fleft"
        linear_x = 0
        angular_z = -0.3
    elif regions["front"] > 1 and regions["fleft"] < 1 and regions["fright"] < 1:
        state_discription = "case-8 fright and fleft"
        linear_x = 0.3
        angular_z = 0
    else:
        state_discription = "unknown!"
        rospy.loginfo(regions)

    rospy.loginfo(state_discription)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)
        

def main():
    
    global pub

    rospy.init_node('obstracal_avoid')

    sub = rospy.Subscriber("/m2wr/laser/scan", LaserScan, clbk_laser)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rospy.spin()
    

if __name__ == "__main__":
    main()