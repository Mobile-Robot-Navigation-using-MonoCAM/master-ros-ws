import rospy
from geometry_msgs.msg import Twist

def run_cmd_callback(data):
    rec_msg = data
    v = rec_msg.linear.x
    w = rec_msg.angular.z

    rospy.loginfo(rospy.get_caller_id() + "v = {}, w = {}".format(v, w))

def cmd_listner():
    rospy.init_node('cmd_listner', anonymous=True)
    rospy.Subscriber('run_cmd', Twist, run_cmd_callback)
    rospy.spin()

if __name__ == '__main__':
    cmd_listner()