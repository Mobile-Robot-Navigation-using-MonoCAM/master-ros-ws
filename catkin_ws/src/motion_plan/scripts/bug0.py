#! /usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

pub_ = None
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None

yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degree
position_ = Point()
desired_position_ = Point()
desired_position_.x = rospy.get_param("des_pos_x")
desired_position_.y = rospy.get_param("des_pos_y")
desired_position_.z = 0

regions_ = None
state_ = 0
state_des_ = ["go to point", "wall follower"]

def laser_clbk(msg):
    global regions_

    regions_ = {
        "right": min(min(msg.ranges[0:143]), 10),
        "fright": min(min(msg.ranges[144:286]), 10),
        "front": min(min(msg.ranges[287:430]), 10),
        "fleft": min(min(msg.ranges[431:574]), 10),
        "left": min(min(msg.ranges[575:719]), 10)
    }

def odom_clbk(msg):
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
    global srv_client_go_to_point_, srv_client_wall_follower_
    log = "State change from [{}] to [{}]".format(state_des_[state_], state_des_[state])
    rospy.loginfo(log)
    state_ = state

    if state == 0:
        res = srv_client_go_to_point_(True)
        res = srv_client_wall_follower_(False)
    elif state == 1:
        res = srv_client_wall_follower_(True)
        res = srv_client_go_to_point_(False)

def normalize_angle(angle):
    if math.fabs(angle) > math.pi :
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def main():
    global regions_, positions_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_

    rospy.init_node("bug_0")

    sub_laser = rospy.Subscriber("/m2wr/laser/scan", LaserScan, laser_clbk)
    sub_odom = rospy.Subscriber("/odom", Odometry, odom_clbk)

    srv_client_go_to_point_ = rospy.ServiceProxy("/go_to_point_switch", SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy("/wall_follower_switch", SetBool)

    change_state(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue

        if state_ == 0 :
            if regions_["front"] < 1:
                change_state(1)
        elif state_ == 1 :
            desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)

            if err_yaw > 0 and math.fabs(err_yaw) < (math.pi/4) and regions_["front"] > 1.5:
                change_state(0)
            if err_yaw > 0 and math.fabs(err_yaw) > (math.pi/4) and math.fabs(err_yaw) < (math.pi/2) and regions_["left"] > 1.5:
                change_state(0)
            if err_yaw < 0 and math.fabs(err_yaw) > (math.pi/4) and math.fabs(err_yaw) < (math.pi/2) and regions_["right"] > 1.5:
                change_state(0)

if __name__ == "__main__":
    main()

