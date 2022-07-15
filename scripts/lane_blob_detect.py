#!/usr/bin/env python3
# Follow white line IRL

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from reu_lane_follow_pkg.cfg import LaneThreshConfig
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

global cf
angle_msg = Float32()
bridge = CvBridge()

def dyn_rcfg_cb(config, level): #get speed, enable, and hsv values from dyn rcfg
  global cf
  cf = config.hue_l, config.hue_h, config.sat_l, config.sat_h, config.val_l, config.val_h
  return config
  
  

def image_callback(ros_image):
  global bridge
  try: 
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
    print(e)
  
  cv_image = cv2.resize(cv_image, None, fx=0.7, fy=0.7, interpolation=cv2.INTER_AREA)
  cv_image = cv2.medianBlur(cv_image, 5) 
  (rows,cols,channels) = cv_image.shape
  cv2.imshow("Original", cv_image)
  cv_image = cv_image[round(rows / 2) : rows, round(cols/2) : cols] #Take bottom left half of image
  
  
  hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  
  tcolLower = (cf[0], cf[2], cf[4]) # Lower bounds of H, S, V for the target color
  tcolUpper = (cf[1], cf[3], cf[5]) # Upper bounds of H, S, V for the target color
  mask = cv2.inRange(hsv_img, tcolLower, tcolUpper)
  
  # find contours in the masked image
  contours,hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
  
  # find the largest contour, max_c
  max_area = 0
  for c in contours:
    M = cv2.moments(c)
    if M['m00'] != 0:
        cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
    area = cv2.contourArea(c)
    if area > max_area:
        max_area = area
        max_c = c
  
  if contours:
    cv2.drawContours(cv_image, max_c, -1, (0,0,255), 10)

  # finding centroids of max contour and draw a circle there
  # https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
  # Handle "div by zero" exceptions caused by unclosed contours
  
  try:
    M = cv2.moments(max_c)
    cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
    cx = cx - 60
  except:
    return     

  # draw a circle at centroid
  # https://www.geeksforgeeks.org/python-opencv-cv2-circle-method/
  cv2.circle(cv_image, (cx,cy), 10, (0,0,0), -1) # -1 fill the circle
  cv2.imshow("RGB image", cv_image)
  cv2.imshow("Masked image", mask)
  cv2.waitKey(3)
  center_point = round(cols * 0.25)
  #rospy.loginfo(f"{center_point}")
  #rospy.loginfo(f"{cx}")
  angle_msg.data = ((center_point - cx) * 0.001)
  #rospy.loginfo(f"{angle_msg.data}")
  angle_pub.publish(angle_msg)



if __name__ == '__main__':
  rospy.init_node('lane_blob_detect', anonymous=True)
  rospy.Subscriber("/camera/image_raw", Image, image_callback)
  angle_pub = rospy.Publisher('/angle', Float32, queue_size=1)
  srv = Server(LaneThreshConfig, dyn_rcfg_cb)
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
