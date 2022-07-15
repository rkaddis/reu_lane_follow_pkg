#!/usr/bin/env python3
# Detect Yellow Blob (Sending synchrously)

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from reu_lane_follow_pkg.cfg import DetectYellowConfig   # packageName.cfg
from geometry_msgs.msg import Twist

###### Global Variables ######
bridge = CvBridge()
global cf
msg = Bool()
##############################

def dyn_rcfg_cb(config, level):
  global cf, min_area
  cf = config.hue_l, config.hue_h, config.sat_l, config.sat_h, config.val_l, config.val_h
  min_area = config.min_area
  return config

def image_callback(ros_image):
  global bridge, cols, rows
  try: #convert ros_image into an opencv-compatible imageadi
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
    print(e)
  cv_image = cv2.medianBlur(cv_image, 5)
  (rows,cols,channels) = cv_image.shape
  #cv2.imshow("Original", cv_image)
  #cv2.waitKey(3)
  cv_image = cv_image[int(rows * 0.5):int(rows * 0.6), int(cols*0.4):int(cols*0.6)] #Take bottom middle of the image
  
  hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
  
  tcolLower = (cf[0], cf[2], cf[4]) # Lower bounds of H, S, V for the target color
  tcolUpper = (cf[1], cf[3], cf[5]) # Upper bounds of H, S, V for the target color
  
  mask = cv2.inRange(hsv_img, tcolLower, tcolUpper)                        
 
  #find contours in the binary (BW) image
  contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
  
  max_area = 0
  max_contour = []

  if len(contours) != 0:
    max_contour = max(contours, key = cv2.contourArea)
    max_area = cv2.contourArea(max_contour)

  #draw the obtained contour lines(or the set of coordinates forming a line) on the original image
  cv2.drawContours(cv_image, max_contour, -1, (0,0,255), 5) # BGR
  cv2.imshow("Yellow Detection Window", cv_image)
  #cv2.imshow("Masked image", mask)
  cv2.waitKey(3)
  rospy.loginfo(f"{max_area}")
  if max_area > min_area:
    msg.data = True
  else:
    msg.data = False         # too small yellow blub or No yellow blub
  
  y_pub.publish(msg)
  
if __name__ == '__main__':
  rospy.init_node('detect_yellow', anonymous=True)
  rospy.Subscriber("/camera/image_raw", Image, image_callback)
  y_pub = rospy.Publisher('yellow_detected', Bool, queue_size=1)
  srv = Server(DetectYellowConfig, dyn_rcfg_cb)
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
