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
g_vel_ramps = [1, 1, 1] # units: meters per second^2
g_stgage = None
g_seat_width=0.2
g_seat_length=0.2
g_stgage_pos_x = [0.0, 0.2, 0.2, 0.0]
g_stgage_pos_y = [0.0, 0.0, 0.2, 0.2]

def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
  # compute maximum velocity step
  step = ramp_rate * (t_now - t_prev).to_sec()
  sign = 1.0 if (v_target > v_prev) else -1.0
  error = math.fabs(v_target - v_prev)
  if error < step: # we can get there within this timestep. we're done.
    return v_target
  else:
    return v_prev + sign * step  # take a step towards the target

def ramped_twist(prev, target, t_prev, t_now, ramps):
    tw = Twist()
    tw.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev,
                            t_now, ramps[2])
    tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev,
                           t_now, ramps[0])
    tw.linear.y = ramped_vel(prev.linear.y, target.linear.y, t_prev,
                           t_now, ramps[1])
    return tw
def calc_vel(stg,stg_x,stg_y,vel_max):
    global g_seat_length, g_seat_width, g_stgage,g_stgage_pos_x,g_stgage_pos_y,g_vel_scales
    tw = Twist()
    tw.x=(stg[0]*stg_x[0]+stg[1]*stg_x[1]+stg[2]*stg_x[2]+stg[3]*stg_x[3])/(stg[0]+stg[1]+stg[2]+stg[3])
    tw.y=(stg[0]*stg_y[0]+stg[1]*stg_y[1]+stg[2]*stg_y[2]+stg[3]*stg_y[3])/(stg[0]+stg[1]+stg[2]+stg[3])#calc Centor Of Gravity Position
    tw.x=tw.x*(g_vel_scales[0]/g_seat_width)
    tw.y=tw.y*(g_vel_scales[1]/g_seat_length)
    return tw

def send_twist():
    global g_last_twist_send_time, g_target_twist, g_last_twist,\
            g_vel_ramps, g_twist_pub
    g_last_twist=calc_vel()
    t_now = rospy.Time.now()
    g_last_twist = ramped_twist(g_last_twist, g_target_twist,
                              g_last_twist_send_time, t_now, g_vel_ramps)
    g_last_twist_send_time = t_now
    g_twist_pub.publish(g_last_twist)

def stgage_cb(msg):
    global g_stgage
    for i in range(4):
        g_stgage[i]=msg.data[i]

def fetch_param(name, default):
  if rospy.has_param(name):
    return rospy.get_param(name)
  else:
    print "parameter [%s] not defined. Defaulting to %.3f" % (name, default)
    return default

if __name__=="__main__":
    rospy.init_node('stgage_twist')
    rospy.Subscriber('stgage',Float32MultiArray,stgage_cb)
    g_twist_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    g_vel_scales[0] = fetch_param('~linear_scale', 1)
    g_vel_scales[1] = fetch_param('~linear_scale', 1)
    g_vel_ramps[2] = fetch_param('~angular_accel', 1)
    g_vel_ramps[0] = fetch_param('~linear_accel', 1)
    g_vel_ramps[1] = fetch_param('~linear_accel', 1)
    g_target_twist = Twist() # initializes to zero
    g_last_twist = Twist()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        send_twist()
        rate.sleep()