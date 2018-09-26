#!/usr/bin/env python
import rospy
import std_msgs.msg import Float32MultiArray
import geometry_msgs.msg import Twist

def stgage_cb(msg):
    msg.data[1]

if __name__=="__main__":
    rospy.init_node('stgage_twist')
    rospy.Subscriber('stgage',Float32MultiArray,stgage_cb)
    g_twist_pub = rospy.Publisher('cmd_vel',Twist)
    