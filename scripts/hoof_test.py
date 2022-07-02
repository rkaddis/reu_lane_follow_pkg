#!/usr/bin/env python3
# Follow a outer line with Yellow Detection

from turtle import color
import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from reu_lane_follow_pkg.cfg import LaneThreshConfig   # packageName.cfg


###### Global Variables ######
angle_msg = Float32()
bridge = CvBridge()
font = cv2.FONT_HERSHEY_SIMPLEX
thresh = 245
###############################
def dyn_rcfg_cb(config, level):
  global thresh1, thresh2, aperture
  thresh1 = config.lower_thresh
  thresh2 = config.upper_thresh
  aperture = 2 * (config.aperture) + 3
  return config



def image_callback(ros_image):
  global bridge, cols, rows, angle_msg
  try: #convert ros_image into an opencv-compatible imageadi
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
    print(e)
  
  (rows,cols,channels) = cv_image.shape
  cv_image = cv_image[round(rows/2) : round(rows), 0 : cols]
  median_image = cv2.medianBlur(cv_image, 5)
  hsv_image = cv2.cvtColor(median_image, cv2.COLOR_BGR2HSV)
  tcolLower = (0, 0, 235)
  tcolUpper = (25, 255, 255)
  mask = cv2.inRange(hsv_image, tcolLower, tcolUpper)
#   gray_image = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
  canny_image = cv2.Canny(mask, thresh1, thresh2, apertureSize = aperture)
  hough_line = cv2.HoughLinesP(canny_image, rho = 6, theta = np.pi /180, threshold = 12, lines = np.array([]), minLineLength = 25, maxLineGap = 15)
  line_image = cv_image.copy()
  # top_pt = (int(cols*0.274), int(rows/4))
  if hough_line is not None:
    r_count = 0
    l_count = 0
    avg_right_slope = 0
    avg_left_slope = 0
    for line in hough_line:
        pt1 = (line[0][0], line[0][1])
        pt2 = (line[0][2], line[0][3])
        cv2.line(line_image, pt1, pt2, 255, 10)
        temp_slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
        if abs(temp_slope) >= 0.01 or abs(temp_slope) <= 100:
          if pt1[0] > round(cols/2) or pt2[0] > round(cols/2):
            right_slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
            avg_right_slope += right_slope
            r_count += 1
          # else:
          #   left_slope = (pt2[1] - pt1[1]) / (pt2[0] -pt1[0])
          #   avg_left_slope += left_slope
          #   l_count += 1
    if r_count == 0:
      r_count = 1
    avg_right_slope = avg_right_slope / r_count
    # avg_left_slope = avg_left_slope / l_count
    avg_slope = (avg_right_slope)*0.8
    if avg_slope >= 150 or avg_slope <= -150:
        if avg_slope >= 150:
            avg_slope = 125
        else:
            avg_slope = -125
    bottom_pt = (int((rows/2)*avg_slope), round(rows/2))
    top_pt = (int((rows/2)*avg_slope), round(rows/4))
    cv2.line(line_image, bottom_pt, top_pt, 150, 10)

    cx = top_pt[0]
    center_point = round(cols * 0.24)
    rospy.loginfo(f"{(center_point - cx) * 0.001}")
    angle_msg.data = ((center_point -cx) * 0.001)
    angle_pub.publish(angle_msg)

  cv2.imshow("mask", mask)
  cv2.imshow("canny image", canny_image)
  cv2.imshow("hough lines w/ source", line_image)
  cv2.waitKey(3)

  #rospy.loginfo(f"{center_point}")
  #rospy.loginfo(f"{angle_msg.data}")
  #blur_kernel = 1 # must be odd, 1, 3, 5, 7 ...
  #bw_img = cv2.medianBlur(bw_img, blur_kernel)                               
  
  #find contours in the binary (BW) image
#   contours, hier = cv2.findContours (bw_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
#   if not contours: # contours do not exists?
#     angle_msg.data = 0
#     velocity_pub.publish(angle_msg)
#     return
  
#   max_area = 0
#   max_c = contours[0]
#   for c in contours:
#     area = cv2.contourArea(c)
#     if area > max_area:
#       max_area = area
#       max_c = c
#   # draw the obtained contour lines(or the set of coordinates forming a line) on the original image
#   # to do: when max_c is null - Not resolved yet
#   if max_c.any() == False: # did not work...
#     angle_msg.data = 0
#     velocity_pub.publish(angle_msg)
#     return
#   else:
#     cv2.drawContours(cv_image, max_c, -1, (0,0,255), 5) # BGR
  
#   # finding centroids of max contour and draw a circle there
#   # https://www.geeksforgeeks.org/python-opencv-find-center-of-contour/
  
#   M = cv2.moments(max_c)
#   try:
#     cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
#     # https://www.geeksforgeeks.org/python-opencv-cv2-circle-method/
#     cv2.circle(cv_image, (cx,cy), 10, (255,0,0), -1) # -1 fill the circle, BGR
    
    
#   except: # div by zero exception may occur due to bad contour
#     cx,cy = rows / 2, cols / 2
#     pass
#   #cv2.imshow("My Image Window", bw_image)
#   cv2.imshow("CV RGB image", cv_image)
#   drive_2_follow_line2(cv_image, cx-10, cy)
  


# def drive_2_follow_line2(cv_image, cx, cy): # algorithm 2
#   global angle_msg
#   mid = cols / 2
#   adj_cx = cx - 220
#   # cx += 200 # to follow left line
#   if adj_cx > mid + 30:
#     angle_msg.data = (mid - adj_cx)/500
#     velocity_pub.publish(angle_msg)
#   elif adj_cx < mid - 30:
#     angle_msg.data = (mid - adj_cx)/500
#     velocity_pub.publish(angle_msg)
#   else:
#     angle_msg.data = 0.0 
#     velocity_pub.publish(angle_msg) 
  

# def stop_vehicle():
#     vel_msg.linear.x = 0.0
#     vel_msg.angular.z = 0.0
#     velocity_pub.publish(vel_msg)
#     return

######## MAIN #######################
if __name__ == '__main__':
  rospy.init_node('lane_follow', anonymous=True)
  # private name
  rospy.Subscriber('/camera/image_raw', Image, image_callback)
  angle_pub = rospy.Publisher('/angle', Float32, queue_size=1)
#   velocity_pub = rospy.Publisher('/angle', Float32, queue_size=1)
  srv = Server(LaneThreshConfig, dyn_rcfg_cb)
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass