#!/usr/bin/env python3
# Follow a outer line with Yellow Detection
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
  global thresh1, thresh2, aperture, cf
  thresh1 = config.lower_thresh
  thresh2 = config.upper_thresh
  aperture = 2 * (config.aperture) + 3
  cf = config.hue_l, config.hue_h, config.sat_l, config.sat_h, config.val_l, config.val_h
  return config


def image_callback(ros_image):
  global bridge, cols, rows, angle_msg

  try: #convert ros_image into an opencv-compatible imageadi
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
    print(e)
  rate = rospy.Rate(20)
  (rows,cols,channels) = cv_image.shape
  cv_image = cv_image[round(rows*2/3) : round(rows), 0 : cols]
  median_image = cv2.medianBlur(cv_image, 5)
  hsv_image = cv2.cvtColor(median_image, cv2.COLOR_BGR2HSV)
  tcolLower = (cf[0], cf[2], cf[4]) # Lower bounds of H, S, V for the target color
  tcolUpper = (cf[1], cf[3], cf[5]) # Upper bounds of H, S, V for the target color
  mask = cv2.inRange(hsv_image, tcolLower, tcolUpper)
  canny_image = cv2.Canny(mask, thresh1, thresh2, apertureSize = aperture)
  hough_line = cv2.HoughLinesP(canny_image, rho = 6, theta = np.pi /180, threshold = 20, lines = np.array([]), minLineLength = 50, maxLineGap = 20)
  line_image = cv_image.copy()
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
          right_slope = (pt2[1] - pt1[1]) / (pt2[0] - pt1[0])
          avg_right_slope += right_slope
          r_count += 1
    try:
      avg_slope = avg_right_slope / r_count
      if avg_slope >= 150 or avg_slope <= -150:
        if avg_slope >= 150:
            avg_slope = 125
        else:
            avg_slope = -125
      bottom_pt = (round(cols*3/4), round(rows/2))
      top_pt = (int((rows*4/7)*avg_slope), round(rows/6))
      cv2.line(line_image, bottom_pt, top_pt, 150, 10)

      cx = top_pt[0]
      rospy.loginfo(f"{(cx)* 0.00035 }")
      angle_msg.data = ((cx)* 0.00035)
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
  srv = Server(LaneThreshConfig, dyn_rcfg_cb)
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass