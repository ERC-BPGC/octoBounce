#!usr/bin/env python3
# https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/


# OpenCV code to track a balls trajectory along with its radius (in pixels)
# Used to track tahe x, y and z position of the ball with 2 cameras placed UNDER the ball and to a side of the ball


import numpy as np
import cv2
import imutils
import time
from collections import deque
import rospy
from geometry_msgs.msg import Twist

orange_lower = (10, .4*255, .6*255)
orange_upper = (30, .8* 255, 1 *255)

ptsUnder = deque(maxlen=64)
ptsOver = deque(maxlen=64)

# def poly(x):  # polynomial function to convert radius of ball to distance from camera
# 	pol = -4.46*10**-6*x**4 + 0.0015*x**3 - 0.183*x**2 + 9.26*x -129.5
# 	return pol

cameraUnder = cv2.VideoCapture(2)
cameraOver = cv2.VideoCapture(4)
# VideCapture(1) is webcam for the left side USB port of Dhruv's laptop - Camera Under the ball
# Camera under the ball is the global origin and it will give us the x and y position of the ball
# VideCapture(2) is webcam for the right side USB port of Dhruv's laptop - Camera to the side of the ball
# Camera to the side of the ball will give us the z position of the ball in the frame of the Camera Under the ball
def ballInfo():
	pub = rospy.Publisher('/ballTracker', Twist, queue_size=10)
	rospy.init_node('ballTracker', anonymous=True)
	rate = rospy.Rate(60) # 60hz

	while not rospy.is_shutdown():
		ballParameter = Twist()
		# (x, y, zArr, velocity, radiusArr) = (0, 0, [], 0, [])
		zArr = []
		t2 = 0 # time to be updated each iteration
		xUnder = 0
		yUnder = 0
		highest = 0

		while True:

			(grabberUnder, frameUnder) = cameraUnder.read()
			(grabberOver, frameOver) = cameraOver.read()

			frameUnder = imutils.resize(frameUnder, width=600)
			frameOver = imutils.resize(frameOver, width=600)

			hsvUnder = cv2.cvtColor(frameUnder, cv2.COLOR_BGR2HSV)
			hsvOver = cv2.cvtColor(frameOver, cv2.COLOR_BGR2HSV)

			maskUnder = cv2.inRange(hsvUnder, orange_lower, orange_upper)
			maskUnder = cv2.erode(maskUnder, None, iterations=2)
			maskUnder = cv2. dilate(maskUnder, None, iterations=2)

			maskOver = cv2.inRange(hsvOver, orange_lower, orange_upper)
			maskOver = cv2.erode(maskOver, None, iterations=2)
			maskOver = cv2. dilate(maskOver, None, iterations=2)

			cntsUnder = cv2.findContours(maskUnder.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
			centerUnder = None

			cntsOver = cv2.findContours(maskOver.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
			centerOver = None

			if len(cntsUnder) > 0:
				# find the largest contour in the mask, then use
				# it to compute the minimum enclosing circle and
				# centroid
				cUnder = max(cntsUnder, key=cv2.contourArea)
				(xUnder, yUnder), radiusUnder = cv2.minEnclosingCircle(cUnder) # x,y coordinates and radius of the minimum enclosing circle
				mUnder = cv2.moments(cUnder)
				centerUnder = (int(mUnder["m10"] / mUnder["m00"]), int(mUnder["m01"] / mUnder["m00"]))

				if radiusUnder > 10:
					cv2.circle(frameUnder, (int(xUnder), int(yUnder)), int(radiusUnder),(0, 255, 255), 2)
					cv2.circle(frameUnder, centerUnder, 5, (0, 0, 255), -1)

			ptsUnder.appendleft(centerUnder)

			if len(cntsOver) > 0:
				# find the largest contour in the mask, then use
				# it to compute the minimum enclosing circle and
				# centroid
				cOver = max(cntsOver, key=cv2.contourArea)
				((xOver, yOver), radiusOver) = cv2.minEnclosingCircle(cOver)
				mOver = cv2.moments(cOver)
				centerOver = (int(mOver["m10"] / mOver["m00"]), int(mOver["m01"] / mOver["m00"]))

				ballParameter.linear.x = xUnder * 0.0264 # 0.026458333333333 is the conversion factor from pix/s to cm/s
				ballParameter.linear.y = yUnder * 0.0264
				ballParameter.linear.z = yOver * 0.0264
				zArr.append(ballParameter.linear.z)
				
				t1 = time.time()
				
				try :
					velocity = (zArr[-1] - zArr[-2])/(t1 - t2)
				except IndexError:
					continue
				
				# velocity = (zArr[-1] - zArr[-2]) / t1 - t2
				t2 = t1	# Update t2 to calculate velocity in next iteration
				# velocityArr.append(velocity)

				ballParameter.angular.z = velocity * 0.026458333333333 

				if radiusOver > 10:
					cv2.circle(frameOver, (int(xOver), int(yOver)), int(radiusOver),(0, 255, 255), 2)
					cv2.circle(frameOver, centerOver, 5, (0, 0, 255), -1)
			ptsOver.appendleft(centerOver)

			# loop over the set of tracked points
			for i in range(1, len(ptsUnder)):
				# if either of the tracked points are None, ignore them
				if ptsUnder[i - 1] is None or ptsUnder[i] is None:
					continue
		
				# otherwise, compute the thickness of the line and
				# draw the connecting lines
				thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
				# cv2.line(frameUnder, ptsUnder[i - 1], ptsUnder[i], (0, 0, 255), thickness) # print the line for balls trajectory

			for i in range(1, len(ptsOver)):
				if ptsOver[i - 1] is None or ptsOver[i] is None:
					continue
				thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
				# cv2.line(frameOver, ptsOver[i - 1], ptsOver[i], (0, 0, 255), thickness)
				# print(xOver, yOver)
			
			
			# show the frame to our screen
			cv2.imshow("FrameUnder", frameUnder)
			cv2.imshow("FrameOver", frameOver)
			key = cv2.waitKey(1) & 0xFF
			if key == ord('q'):
				break
			

			if velocity == highest - 0.098/2:
				rospy.loginfo(ballParameter)
				pub.publish(ballParameter)
				rate.sleep()
			


		cameraUnder.release()
		cameraOver.release()
		cv2.destroyAllWindows()


if __name__ == '__main__':
	try:
		ballInfo()
	except rospy.ROSInterruptException:
		pass
