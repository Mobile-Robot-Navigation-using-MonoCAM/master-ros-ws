#! /usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# parameters
pub = None

regions_ = {
    "right": 0,
    "fright": 0,
    "front": 0,
    "fleft": 0,
    "left": 0
}
state_ = 0

state_dct_ = {
    0: "find wall",
    1: "turn left",
    2: "follow the wall"
}

def change_state(state):
    global state_
    if state is not state_:
        print("State change from [{}] to [{}]".format(state_dct_[state_], state_dct_[state]))
        state_ = state

def take_action():
    global regions_
    regions = regions_

    state_discription = ""

    d = 1.5
    
    if regions["front"] > d and regions["fleft"] > d and regions["fright"] > d:
        state_discription = "case-1 none"
        change_state(0)
        
    elif regions["front"] < d and regions["fleft"] > d and regions["fright"] > d:
        state_discription = "case-2 front"
        change_state(1)

    elif regions["front"] > d and regions["fleft"] > d and regions["fright"] < d:
        state_discription = "case-3 front_right"
        change_state(2)

    elif regions["front"] > d and regions["fleft"] < d and regions["fright"] > d:
        state_discription = "case-4 front_left"
        change_state(0)

    elif regions["front"] < d and regions["fleft"] > d and regions["fright"] < d:
        state_discription = "case-5 front and fright"
        change_state(1)

    elif regions["front"] < d and regions["fleft"] < d and regions["fright"] > d:
        state_discription = "case-6 front and fleft"
        change_state(1)

    elif regions["front"] < d and regions["fleft"] < d and regions["fright"] < d:
        state_discription = "case-7 front and fright and fleft"
        change_state(1)

    elif regions["front"] > d and regions["fleft"] < d and regions["fright"] < d:
        state_discription = "case-8 fright and fleft"
        change_state(0)

    else:
        state_discription = "unknown case!"
        rospy.loginfo(regions)
    


def clbk_laser(msg):
    global regions_
    regions_ = {
        "right": min(min(msg.ranges[0:143]), 10),
        "fright": min(min(msg.ranges[144:286]), 10),
        "front": min(min(msg.ranges[287:430]), 10),
        "fleft": min(min(msg.ranges[431:574]), 10),
        "left": min(min(msg.ranges[575:719]), 10)
    }
    
    take_action()

def find_wall():
    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = 0.3
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = -0.3
    return msg

def follow_the_wall():
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def main():
    global pub, state_

    rospy.init_node("wall_follower")
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    sub_laser = rospy.Subscriber("/m2wr/laser/scan", LaserScan, clbk_laser)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub.publish(msg)

        rate.sleep()

if __name__ == "__main__":
    main()