#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

g_twist_pub = None
g_target_twist = None 
g_last_twist = None
g_last_send_time = None
g_vel_scales = [0.1, 0.1, 0.1]
g_vel_ramps = [1, 1, 1] # units: meters per second^2 [x,y,z]
g_vel_max = [0.0,0.0,0.0] #units: meter per second [x,y,z] 
g_cogpos = Float32MultiArray()#Centor Of Gravity Position on seat [x,y]

def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
  # compute maximum velocity step
  step = ramp_rate * (t_now - t_prev).to_sec()
  sign = 1.0 if (v_target > v_prev) else -1.0
  error = math.fabs(v_target - v_prev)
  if error < step: # we can get there within this timestep. we're done.
    return v_target
  else:
    return v_prev + sign * step  # take a step towards the target

def ramped_twist(prev, target, t_prev, t_now, ramps, max):
    tw = Twist()
  #  tw.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev,
  #                          t_now, ramps[2])
    tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev,
                           t_now, ramps[0])
    tw.linear.y = ramped_vel(prev.linear.y, target.linear.y, t_prev,
                           t_now, ramps[1])
    return tw

def calc_vel(cogpos,vel_scales,vel_max):
    tw = Twist()
    tw.linear.x=cogpos.data[0]*vel_scales[0]
    tw.linear.y=-cogpos.data[1]*vel_scales[1]
    if tw.linear.x > vel_max[0]:
       tw.linear.x = vel_max[0]
    elif tw.linear.x < -vel_max[0]:
      tw.linear.x = -vel_max[0]
  
    if tw.linear.y > vel_max[1]:
      tw.linear.y = vel_max[1]
    elif tw.linear.y < -vel_max[1]:
      tw.linear.y = -vel_max[1]
    return tw

def send_twist():
    global g_last_twist_send_time, g_target_twist, g_last_twist,\
            g_vel_ramps, g_twist_pub,g_cogpos,g_vel_scales,g_vel_max
    g_target_twist=calc_vel(g_cogpos,g_vel_scales,g_vel_max)
    t_now = rospy.Time.now()
    g_last_twist = ramped_twist(g_last_twist, g_target_twist,
                              g_last_twist_send_time, t_now, g_vel_ramps,g_vel_max)
    g_last_twist_send_time = t_now
    g_twist_pub.publish(g_last_twist)

def cogpos_cb(msg):
    global g_cogpos
    g_cogpos = msg
    send_twist()

def fetch_param(name, default):
  if rospy.has_param(name):
    return rospy.get_param(name)
  else:
    print "parameter [%s] not defined. Defaulting to %.3f" % (name, default)
    return default

if __name__=="__main__":
    rospy.init_node('cogpos_twist')
    g_vel_scales[0] = fetch_param('~linear_scale_x', 1)
    g_vel_scales[1] = fetch_param('~linear_scale_y', 1)
    g_vel_ramps[2] = fetch_param('~angular_accel', 1)
    g_vel_ramps[0] = fetch_param('~linear_accel_x', 1)
    g_vel_ramps[1] = fetch_param('~linear_accel_y', 1)
    g_vel_max[0] = fetch_param('~linear_maxspeed_x',0.1)
    g_vel_max[1] = fetch_param('~linear_maxspeed_y',0.1)
    g_vel_max[2] = fetch_param('~angular_maxspeed_z',0.1)
    g_last_twist_send_time = rospy.Time.now()
    g_target_twist = Twist() # initializes to zero 
    g_last_twist = Twist()
    rospy.Subscriber('cogpos',Float32MultiArray,cogpos_cb)
    g_twist_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    rospy.spin()