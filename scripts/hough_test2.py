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
  global thresh1, thresh2, aperture, rho, thresh_count, min_length, max_gap
  thresh1 = config.lower_thresh
  thresh2 = config.upper_thresh
  aperture = 2 * (config.aperture) + 3
  rho = config.rho
  thresh_count = config.line_thresh
  min_length = config.line_length
  max_gap = config.line_gap
  return config



def image_callback(ros_image):
  global bridge, cols, rows, angle_msg

  try: #convert ros_image into an opencv-compatible imageadi
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
    print(e)
  rate = rospy.Rate(20)
  (rows,cols,channels) = cv_image.shape
  cv_image = cv_image[round(rows*7/10) : round(rows), round(cols/10) : cols]
  median_image = cv2.medianBlur(cv_image, 5)
  hsv_image = cv2.cvtColor(median_image, cv2.COLOR_BGR2HSV)
  tcolLower = (0, 0, 235)
  tcolUpper = (1, 10, 255)
  mask = cv2.inRange(hsv_image, tcolLower, tcolUpper)
  canny_image = cv2.Canny(mask, thresh1, thresh2, apertureSize = aperture)
  hough_line = cv2.HoughLinesP(canny_image, rho = rho, theta = np.pi /180, threshold = thresh_count, lines = np.array([]), minLineLength = min_length, maxLineGap = max_gap)
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
          # if pt1[0] > round(cols/3) or pt2[0] > round(cols/3):
            right_slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
            avg_right_slope += right_slope
            r_count += 1
        # else:
        #   r_count += 1
          # else:
          #   left_slope = (pt2[1] - pt1[1]) / (pt2[0] -pt1[0])
          #   avg_left_slope += left_slope
          #   l_count += 1
    try:
      avg_right_slope = avg_right_slope / r_count
    # avg_left_slope = avg_left_slope / l_count
      avg_slope = (avg_right_slope)
      if avg_slope >= 150 or avg_slope <= -150:
        if avg_slope >= 150:
            avg_slope = 125
        else:
            avg_slope = -125
      bottom_pt = (round(cols*3/4), round(rows/2))
      top_pt = (int((rows*4/7)*avg_slope), round(rows/6))
      cv2.line(line_image, bottom_pt, top_pt, 150, 10)

      cx = top_pt[0]
      center_point = round(cols * 0.245)
      rospy.loginfo(f"{(cx)* 0.00037 }")
      angle_msg.data = ((cx)* 0.00037)
      angle_pub.publish(angle_msg)
    except:
      pass
  cv2.imshow("mask", mask)
  cv2.imshow("canny image", canny_image)
  cv2.imshow("hough lines w/ source", line_image)
  cv2.waitKey(3)
  rate.sleep()

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
