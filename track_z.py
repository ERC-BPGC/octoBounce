# https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/


# OpenCV code to track a balls trajectory along with its radius (in pixels)
# Used to track the z position of the ball with a camera placed BESIDE it


import numpy as np
import cv2
import imutils
# import time
from collections import deque
orange_lower = (10, .4*255, .6*255)
orange_upper = (30, .8* 255, 1 *255)
pts = deque(maxlen=64)

# def poly(x):  # polynomial function to convert radius of ball to distance from camera
# 	pol = -4.46*10**-6*x**4 + 0.0015*x**3 - 0.183*x**2 + 9.26*x -129.5
# 	return pol

# def findAvg(x):
# 	avg = 0
# 	for i in range(len(x)):
# 		avg += x[i]
# 	avg = avg/len(x)
# 	return avg
# radiusArr = [] 
# polyArr = []

camera = cv2.VideoCapture(1)
# VideCapture(1) is webcam for the left side USB port of Dhruv's laptop
# VideCapture(2) is webcam for the right side USB port of Dhruv's laptop

while True:

	(grabber, frame) = camera.read()

	frame = imutils.resize(frame, width=600)

	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	mask = cv2.inRange(hsv, orange_lower, orange_upper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2. dilate(mask, None, iterations=2)

	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None

	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid

		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c) # x,y coordinates and radius of the minimum enclosing circle
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		if radius > 10:
			cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
	pts.appendleft(center)

	# loop over the set of tracked points
	for i in range(1, len(pts)):
		# if either of the tracked points are None, ignore them
		if pts[i - 1] is None or pts[i] is None:
			continue
		
	

		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness) # print the line for balls trajectory

		z = y # y-coordinate of camera is z-coordinate in ball frame because of the orientation of camera
		# print(z)


		# if(radius != 0 and poly(radius) != 0): # find moving average of the radius of the ball and the depth of the ball
		# 	radiusArr.append(radius)
		# 	polyArr.append(poly(radius))

		# print(findAvg(radiusArr), findAvg(polyArr))
		
	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		break


# rsum = 0
# for x in radiusArr:

# 	rsum = rsum + x

# print("radius = ", rsum/len(radiusArr)) # Print the average radius over the complete video

# psum = 0
# for y in polyArr:
# 	psum = psum + y

# print("depth = ", psum/len(polyArr))


camera.release()
cv2.destroyAllWindows()
