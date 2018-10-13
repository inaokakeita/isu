#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

g_cogpos_pub = None
g_stg = [0.01, 0.01, 0.01, 0.01] #stgage sensor raw data
g_cog_offset = [0.0,0.0]
g_stg_x = [0.0, 0.2, 0.2, 0.0]#straigngage sensor position 
g_stg_y = [0.0, 0.0, 0.2, 0.2]
g_rate = 20
g_stg_init_flg = True

#todo stop moving at buttons pushed
def calc_cog(stg,stg_x,stg_y):
    cog[0] =(stg[0]*stg_x[0]+stg[1]*stg_x[1]+stg[2]*stg_x[2]+stg[3]*stg_x[3])/(stg[0]+stg[1]+stg[2]+stg[3])
    cog[1]=(stg[0]*stg_y[0]+stg[1]*stg_y[1]+stg[2]*stg_y[2]+stg[3]*stg_y[3])/(stg[0]+stg[1]+stg[2]+stg[3])#calc Centor Of Gravity Position
    return cog

def send_cogpos():
    global g_stg,g_stg_x,g_stg_y,g_joy,g_cog_offset
    cogpos=calc_vel(g_stg,g_stg_x,g_stg_y)
    if g_stg_init_flg == True:
        g_cog_offset = cogpos
        cogpos = [0.0, 0.0]
    else:
        cogpos = cogpos - g_cog_offset
    g_cogpos_pub.publish(cogpos)

def stgage_cb(msg):
    global g_stg
    for i in range(4):
        g_stg[i]=msg.data[i]

def joy_cb(msg):
    global g_stg_init_flg
    if g_joy.buttons[4]==1 and g_joy.buttons[5]==1:
        g_stg_init_flg = not(g_stg_init_flg)
        rospy.sleep(0.5)
      

def fetch_param(name, default):
  if rospy.has_param(name):
    return rospy.get_param(name)
  else:
    print "parameter [%s] not defined. Defaulting to %.3f" % (name, default)
    return default

if __name__=="__main__":
    rospy.init_node('stgage_cogpos')
    rospy.Subscriber('stgage',Float32MultiArray,stgage_cb)
    rospy.Subscriber('joy',Joy,joy_cb)
    g_cogpos_pub = rospy.Publisher('cogpos',Float32MultiArray,queue_size=1)
    g_rate = fetch_param('~rate',20)
    rate = rospy.Rate(g_rate)
    while not rospy.is_shutdown():
        send_cogpos()
        rate.sleep()