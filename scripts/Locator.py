import numpy as np;
import cv2;


def getAngle(x1,x2,y1,y2):
	dx = x2 - x1
	dy = y2 - y1
	rads = np.arctan2(dx,dy)
#	if rads < 0:
#		rads = rads + 2* np.pi
	return rads
def calculateDistance(x1,y1,x2,y2):
     dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
     return dist

def getRobotPos(img):
	distanceBetweenMarkers = .24 #Distance between markers in meters
	params = cv2.SimpleBlobDetector_Params()

	params.minThreshold = 10;
	params.maxThreshold = 200;
	# Set up the detector with default parameters.
        #detector = cv2.SimpleBlobDetector_create() #King, 29FEB2016 8:00PM changed from this to line below
        detector = cv2.SimpleBlobDetector(params) 

	#print(hsv_green[0][0][0]-20,hsv_green[0][0][1], hsv_green[0][0][2]-20)
        greenLower = (40,100,100)
        greenUpper = (80,255,255)

        greenLower = (40,50,50)
        greenUpper = (80,255,255)


	blue_lower= (100,150,0)
	blue_upper=(140,255,255)

	red_lower= (5,50,50)
	red_upper=(20,255,255)

	count = 0
	# Capture frame-by-frame
	frame = img
	# Our operations on the frame come here
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	#hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)
	mask_blue = cv2.erode(mask_blue, None, iterations=2)
	mask_blue = cv2.dilate(mask_blue, None, iterations=2)

	mask_red = cv2.inRange(hsv, red_lower, red_upper)
	mask_red = cv2.erode(mask_red, None, iterations=2)
	mask_red = cv2.dilate(mask_red, None, iterations=2)


	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None

	cnts_blue = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None

	cnts_green = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None
	# Detect blobs.
	#keypoints_blue = detector.detect(res_blue)
	keypoints = detector.detect(mask)
	green_x = 0
	green_y = 0
	blue_x =  0
	blue_y =  0

	#Green
	if len(cnts_green) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x_end, y_end), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x_end), int(y_end)), int(radius),
				(0, 255, 255), 2)
			green_x = int(x_end)
			green_y = int(y_end)
			#cv2.circle(frame, center, 5, (0, 0, 255), -1)
			#print("Green is at " + str(x_end) + " , " + str(y_end) + "\n")
	#blue
	if len(cnts_blue) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts_blue, key=cv2.contourArea)
		((x_robot, y_robot), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center_blue = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x_robot), int(y_robot)), int(radius),
				(0, 255, 255), 2)
			#cv2.circle(frame, center, 5, (0, 0, 255), -1)
			#print("The Blue point is at " + str(x_robot) + " , " + str(y_robot) + "\n\n")
			blue_x = int(x_robot)
			blue_y = int(y_robot)
	x_pos = (max(green_x,blue_x) + min(green_x,blue_x))/2
	y_pos = (max(green_y,blue_y) + min(green_y,blue_y))/2
	cv2.circle(frame, (int(x_pos),int(y_pos)), 5, (0, 0, 255), -1)
	angleInRads = getAngle(green_x,blue_x,green_y,blue_y)
	#print("Robot X:" + str(x_pos) + " Y: " + str(y_pos))
	height = np.size(frame, 0)
	width = np.size(frame, 1)
	#im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	#cv2.imshow('keys2',mask)
	#cv2.imshow('Blue Display', mask_blue);
	pixelsBetweenMarkers = calculateDistance(green_x,green_y,blue_x,blue_y)
	pixelsPerMeter = pixelsBetweenMarkers/distanceBetweenMarkers
	return [x_pos,y_pos,angleInRads,pixelsPerMeter]
	
def getTargetPos(img):#Returns position, and also all of the indices of the target so that Driver can mark them as known driveable.
        yellow_lower= (10,100,100)
        yellow_upper=(30,255,255)
	params = cv2.SimpleBlobDetector_Params()

	params.minThreshold = 10;
	params.maxThreshold = 200;
	# Set up the detector with default parameters.
        #detector = cv2.SimpleBlobDetector_create() #King, 29FEB2016 8:00PM changed from this to line below
        detector = cv2.SimpleBlobDetector(params) 
	count = 0
	# Capture frame-by-frame
	frame = img
	# Our operations on the frame come here
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	#hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)
	mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
	mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None
	keypoints = detector.detect(mask_yellow)
        yellow_x=0
        yellow_y=0
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x_end, y_end), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x_end), int(y_end)), int(radius),
				(0, 255, 255), 2)
			yellow_x = int(x_end)
			yellow_y = int(y_end)
			#cv2.circle(frame, center, 5, (0, 0, 255), -1)
                        return (yellow_x,yellow_y,radius)
