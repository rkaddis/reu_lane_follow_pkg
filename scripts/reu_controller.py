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
yellow = True
lane_follow = False
approaching_line = False
first_run = True


def dyn_rcfg_cb(config, level):
    global enable, speed, inside_lane
    speed = config.speed
    enable = config.enable
    inside_lane = config.inside_lane
    return config

def yellow_cb(msg):
    global yellow
    yellow = msg.data
    
def angle_cb(msg):
    global speed, enable, vel_msg, empty_msg, yellow, lane_follow, approaching_line, inside_lane, first_run, ignore_yellow
    if(enable):
        
        if(lane_follow): #do lane following and watch for line
            if rospy.Time.now().to_sec() < ignore_yellow:
                yellow = False
            vel_msg.linear.x = speed
            vel_msg.angular.z = msg.data
            
            # if(yellow and inside_lane):
            # 	# vel_msg.angular.z = -0.25
            
            if(approaching_line and not yellow):
                #yellow line leaves camera view
                lane_follow = False
            
            approaching_line = yellow
            rospy.loginfo(f"{approaching_line}")
            
        else: #dead reckon turn once yellow is lost
            #pull straight forward to line
            drive_to_line = rospy.Time.now().to_sec() + 3.5
            if(inside_lane):
            	drive_to_line -=0.1
            if(not first_run):	
            	while(enable and drive_to_line > rospy.Time.now().to_sec()):
               	 vel_msg.linear.x = speed
               	 vel_msg.angular.z = 0
            
            #stop for 3 seconds    
            wait_time = rospy.Time.now().to_sec() + 3
            while(enable and wait_time > rospy.Time.now().to_sec()):
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
            
            #do dead reckon turn
            dead_yaw = 0.1
            turn_time = rospy.Time.now().to_sec() + 19
            ignore_yellow = turn_time + 16
            if(inside_lane):
                turn_time -= 5
                dead_yaw = -0.167
                turn_time = turn_time - 0
            while(enable and turn_time > rospy.Time.now().to_sec()):
                vel_msg.linear.x = speed
                vel_msg.angular.z = dead_yaw
            first_run = False
            lane_follow = True #re-enable lane follow

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
    rospy.Subscriber('yellow_detected', Bool, yellow_cb)
    srv = Server(REUControllerConfig, dyn_rcfg_cb)
    publish_ctrl()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    
    
    
    
    
    
    
