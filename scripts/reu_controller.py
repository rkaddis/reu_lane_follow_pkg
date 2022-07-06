#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from dynamic_reconfigure.server import Server
from reu_lane_follow_pkg.cfg import REUControllerConfig

vel_msg = Twist()
empty_msg = Empty()


def dyn_rcfg_cb(config, level):
    global enable, speed
    speed = config.speed
    enable = config.enable
    return config
    
def angle_cb(msg):
    global speed, enable, vel_msg, empty_msg
    if(enable):
        vel_msg.linear.x = speed
        vel_msg.angular.z = msg.data
        
    else:
       vel_msg.linear.x = 0
       vel_msg.angular.z = 0
         
        
def publish_ctrl():
    global empty_msg, vel_msg, enable
    rate = rospy.Rate(20)
    while(not rospy.is_shutdown()):
        if(enable):
            empty_pub.publish(empty_msg)
            vel_pub.publish(vel_msg)
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            empty_pub.publish(empty_msg)
            vel_pub.publish(vel_msg)
        rate.sleep()
        
        
        
if __name__ == '__main__':
    rospy.init_node('reu_controller', anonymous = True)
    empty_pub = rospy.Publisher('/vehicle/enable', Empty, queue_size=1)
    vel_pub = rospy.Publisher('/vehicle/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/angle', Float32, angle_cb)
    srv = Server(REUControllerConfig, dyn_rcfg_cb)
    publish_ctrl()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    
    
    
    
    
    
    
