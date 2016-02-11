#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

from sensor_msgs.msg import Joy

g_cogpos_pub = None
g_stg = [0.01, 0.01, 0.01, 0.01] #stgage sensor raw data
g_cog_offset = Float32MultiArray()
g_cog_offset.data = [0.0,0.0]
g_stg_x = [0.0, 0.0, 0.253, 0.253]#straigngage sensor position 
g_stg_y = [0.0, 0.253, 0.0, 0.253]
g_rate = 20
g_stg_init_flg = True
g_joy = Joy()

def calc_cog(stg,stg_x,stg_y):
    cog = Float32MultiArray()
    cog.data=[0.0,0.0]
    if  (stg[0]+stg[1]+stg[2]+stg[3]) != 0:
        cog.data[0] =(stg[0]*stg_x[0]+stg[1]*stg_x[1]+stg[2]*stg_x[2]+stg[3]*stg_x[3])/(stg[0]+stg[1]+stg[2]+stg[3])
        cog.data[1] =(stg[0]*stg_y[0]+stg[1]*stg_y[1]+stg[2]*stg_y[2]+stg[3]*stg_y[3])/(stg[0]+stg[1]+stg[2]+stg[3])#calc Centor Of Gravity Position
    return cog

def send_cogpos():
    global g_stg,g_stg_x,g_stg_y,g_cog_offset,g_stg_init_flg
    cogpos=calc_cog(g_stg,g_stg_x,g_stg_y)
    if g_stg_init_flg is True:
        g_cog_offset.data = cogpos.data
        cogpos.data = [0.0, 0.0]
    else:
        cogpos.data[0] = cogpos.data[0] - g_cog_offset.data[0]
        cogpos.data[1] = cogpos.data[1] - g_cog_offset.data[1]
    g_cogpos_pub.publish(cogpos)

def stgage_cb(msg):
    global g_stg
    for i in range(4):
        g_stg[i]=msg.data[i]

def joy_cb(msg):
    global g_stg_init_flg
    if msg.buttons[4]==1 and msg.buttons[5]==1:
        if g_stg_init_flg is False:
            rospy.logwarn('stop')
            g_stg_init_flg = True
    if msg.buttons[6]==1 and msg.buttons[7]==1:
        if g_stg_init_flg is True:
            rospy.logwarn('start')
            g_stg_init_flg = False

def fetch_param(name, default):
  if rospy.has_param(name):
    return rospy.get_param(name)
  else:
    print "parameter [%s] not defined. Defaulting to %.3f" % (name, default)
    return default

if __name__=="__main__":
    rospy.init_node('stgage_cogpos')
    rospy.Subscriber('stg_array',Int32MultiArray,stgage_cb)
    rospy.Subscriber('joy',Joy,joy_cb)
    g_cogpos_pub = rospy.Publisher('cogpos',Float32MultiArray,queue_size=1)
    g_rate = fetch_param('~rate',20)
    rate = rospy.Rate(g_rate)
    while not rospy.is_shutdown():
        send_cogpos()
        rate.sleep()