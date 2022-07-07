# reu_lane_follow_pkg

Package for lane following and evaluation for LTU/MSU REU project
Clone this repo: $ git clone https://github.com/rkaddis/reu_lane_follow_pkg

<b> Description: </b> <br>

<b> 1. Blob Detection and Shifted Line Following: </b> <br>
The camera view detects the left lane line by using HSV filtering and blob detection. <br>
The car then uses the center of the blob to determine the center of the lane, and angles itself to stay in the lane.
<br> <br>

<b> 2. Canny/Hough Line Detection with Spring Method Lane Centering </b> <br>
The camera view uses Canny edge detection to create contrast between lane lines and the road,
then uses Hough line detection to overlay virtual lines on the camera image. <br>
Then, using a number of springs connected to the Hough lines, the center of the lane can be calculated as the 
springs push and pull on the center point. <br>
The car then angles towards the calculated center point.
<br> <br>

<b> 3. Canny/Hough Line Detection with Cubic Spline Lane Calculation </b> <br>
Like the previous algorithm, the Canny/Hough method is used for lane line detection, then a ficticious 
center line is formed by a cubic spline. <br>
The car can then follow the virtual line. <br>
