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
g_vel_max = None #units: meter per second
g_angular_max = None
g_cogpos = Float32MultiArray()#Centor Of Gravity Position on seat [x,y]
g_deadzone = None

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
    tw.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev,
                            t_now, ramps[2])
    tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev,
                           t_now, ramps[0])
    tw.linear.y = ramped_vel(prev.linear.y, target.linear.y, t_prev,
                           t_now, ramps[1])
    return tw

def calc_vel(cogpos,vel_scales,vel_max,angular_max,deadzone):
    tw = Twist()
    #mapping cogpos to velocity
    tw.linear.x=cogpos.data[0]*vel_scales[0]
    tw.linear.y=-cogpos.data[1]*vel_scales[1]
    #convert cartesian to polar coordinate
    vel_angle = math.atan2(tw.linear.y,tw.linear.x)
    vel_r = math.sqrt(math.pow(tw.linear.x,2)+math.pow(tw.linear.y,2))
    #set maxspeed
    if vel_r > vel_max:
      vel_r = vel_max
    #set dead one
    if vel_r < deadzone:
        vel_r = 0.0
    #mapping rotate
    if vel_angle > math.pi/2:
      tw.angular.z = -vel_r * vel_scales[2]
      if tw.angular.z < -angular_max:
        tw.angular.z = -angular_max
    elif  vel_angle < -math.pi/2:
      tw.angular.z = vel_r * vel_scales[2]
      if tw.angular.z > angular_max:
        tw.angular.z = angular_max
    #forbid back
    if vel_angle > math.pi/2 or vel_angle < -math.pi/2:
      vel_r = 0.0
    #convert polar to cartesian coordinate
    tw.linear.x = math.cos(vel_angle)*vel_r
    tw.linear.y = math.sin(vel_angle)*vel_r
    return tw

def send_twist():
    global g_last_twist_send_time, g_target_twist, g_last_twist,\
            g_vel_ramps, g_twist_pub,g_cogpos,g_vel_scales,g_vel_max,g_deadzone,g_angular_max
    g_target_twist=calc_vel(g_cogpos,g_vel_scales,g_vel_max,g_angular_max,g_deadzone)
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
    g_vel_scales[2] = fetch_param('~angular_scale_z',1)
    g_vel_ramps[0] = fetch_param('~linear_accel_x', 1)
    g_vel_ramps[1] = fetch_param('~linear_accel_y', 1)
    g_vel_ramps[2] = fetch_param('~angular_accel_z', 1)
    g_vel_max = fetch_param('~linear_maxspeed',0.1)
    g_angular_max = fetch_param("~angular_maxspeed",1.0)
    g_deadzone = fetch_param('~deadzone',0.1)
    g_last_twist_send_time = rospy.Time.now()
    g_target_twist = Twist() # initializes to zero 
    g_last_twist = Twist()
    rospy.Subscriber('cogpos',Float32MultiArray,cogpos_cb)
    g_twist_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    rospy.spin()