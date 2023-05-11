#! /usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations

from std_srvs.srv import *

import math

# robot_state_variables
position_ = Point()
yaw_ = 0
# machine_state
state_ = 0
# glob
desired_position_ = Point()
# get desired point from user:
desired_position_.x = rospy.get_param("des_pos_x")
desired_position_.y = rospy.get_param("des_pos_y")
desired_position_.z = 0

# parameters
yaw_precision_ = math.pi/90 # about 2 degree of precision
dist_precision_ = 0.3

# publishers
pub_ = None
active_ = False

def go_to_service_handler(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = "Done !"
    return res


# callbacks
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
        )
    euler = transformations.euler_from_quaternion(quaternion)
    # roll = euler[0]
    # pitch = euler[1]
    yaw_ = euler[2]

def change_state(state):
    global state_
    state_ = state
    print("State changed to : {}".format(state_))

def fix_yaw(des_pos):
    global yaw_, pub_, yaw_precision_, state_

    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        # print("Yaw error: [{}]".format(math.fabs(err_yaw)),end="=")
        # print("Yaw precision: [{}]".format(yaw_precision_),end="=")
        twist_msg.angular.z = -0.7 if err_yaw > 0 else 0.7
        # print(twist_msg.angular.z)

    pub_.publish(twist_msg)

    # state condition
    if math.fabs(err_yaw) <= yaw_precision_:
        print("Yaw error: [{}]".format(err_yaw))
        change_state(1)

def go_straight_ahead(des_pos):
    global yaw_, pub_, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        # print("Distance error: [{}]".format(err_pos))
        twist_msg.linear.x = 0.6
        pub_.publish(twist_msg)
    else:
        # change state condition
        print("Distance error: [{}]".format(err_pos))
        change_state(2)
    
    # change state according to yaw_error
    if math.fabs(err_yaw) > yaw_precision_:
        print("Yaw error: [{}]".format(err_yaw))
        change_state(0)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)


def main():
    global pub_

    rospy.init_node("go_to_point")

    pub_ = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    sub_odom = rospy.Subscriber("/odom", Odometry, clbk_odom)

    rospy.Service("go_to_point_switch", SetBool, go_to_service_handler)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done()
            else:
                rospy.logerr("unknown state")
        
        rate.sleep()


if __name__ == "__main__":
    main()