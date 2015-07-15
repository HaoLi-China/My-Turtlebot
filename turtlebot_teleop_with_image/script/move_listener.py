#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def callback(data):
    cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    # Twist is a datatype for velocity
    move_cmd = Twist()
    # let's go forward at 0.2 m/s
    if data.data == 'forward':
        move_cmd.linear.x = 0.2
        # let's turn at 0 radians/s
        move_cmd.angular.z = 0
    elif data.data == 'back':
        move_cmd.linear.x = -0.2
        # let's turn at 0 radians/s
        move_cmd.angular.z = 0
    elif data.data == 'left':
        move_cmd.angular.z = 0.1
    elif data.data == 'right':
        move_cmd.angular.z = -0.1

    cmd_vel.publish(move_cmd)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("move_topic", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

