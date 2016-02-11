#! /usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

g_cart_angle = None
g_robot_radius = None
g_robot_tread = None
g_wheel_radius = None
g_rate = None

def ik_solver(vel,cart_angle,radius,tread,wheelr):
    global g_cart_angle,g_rate
    w_perimeter = math.pi*2*wheelr
    l_spd = vel.linear.x * ((math.cos(cart_angle)-tread/radius * math.sin(cart_angle)) / w_perimeter) + vel.linear.y * ((math.sin(cart_angle)+tread/radius * math.cos(cart_angle))/w_perimeter)
    r_spd = vel.linear.x * ((math.cos(cart_angle)+tread/radius * math.sin(cart_angle)) / w_perimeter) + vel.linear.y * ((math.sin(cart_angle)-tread/radius * math.cos(cart_angle))/w_perimeter)
    w_spd = vel.linear.x * (-math.sin(cart_angle) / radius) + vel.linear.y * (math.cos(cart_angle) / radius) + vel.angular.z
    g_cart_angle += w_spd / g_rate
    if g_cart_angle >= 2*math.pi:
        g_cart_angle -= 2 * math.pi
    elif g_cart_angle <= -2 * math.pi:
        g_cart_angle += 2 * math.pi
    axes_vel = [l_spd,r_spd,-w_spd]
    return axes_vel

def twist_cb(vel,float_pub):
    global g_cart_angle,g_robot_radius,g_robot_tread,g_wheel_radius
    float_spds = ik_solver(vel,g_cart_angle,g_robot_radius,g_robot_tread,g_wheel_radius)
    float_pub.publish(data = float_spds)

def fetch_param(name,default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return default
        
if __name__=='__main__':
    rospy.init_node('ik_solver')
    float_pub = rospy.Publisher('axes_vel',Float32MultiArray,queue_size=1)
    rospy.Subscriber('cmd_vel',Twist,twist_cb,float_pub)
    g_cart_angle = 0.0
    g_robot_radius = fetch_param('~active_caster_radius',0.139625)
    g_robot_tread = fetch_param('~active_caster_half_tread',0.139625)
    g_wheel_radius = fetch_param('~wheel_radius',0.05)
    g_rate = fetch_param('~rate',20)
    rospy.spin()
