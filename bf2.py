import numpy as np
import cv2

cap = cv2.VideoCapture(0)

params = cv2.SimpleBlobDetector_Params()

params.minThreshold = 10;
params.maxThreshold = 200;
# Set up the detector with default parameters.
detector = cv2.SimpleBlobDetector_create() 
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)

blue_lower= (100,150,0)
blue_upper=(140,255,255)





cv2.waitKey(0)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()


    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)
    mask_blue = cv2.erode(mask_blue, None, iterations=2)
    mask_blue = cv2.dilate(mask_blue, None, iterations=2)



    # find contours in the mask and initialize the current
	# (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    cnts_blue = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL,
    	cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    #mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    #mask_blue = cv2.erode(mask_blue, None, iterations=3)
    #mask_blue = cv2.dilate(mask_blue, None, iterations=2)
    #res_blue = cv2.bitwise_and(frame,frame, mask= mask_blue)
    #blur = cv2.blur(mask,(5,5))
    # Detect blobs.
    #keypoints_blue = detector.detect(res_blue)
    keypoints = detector.detect(mask)
    #contours, hierarchy = cv2.findContours(blur,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    x_robot = 1000
    y_robot = 1000
    x_end = 0
    y_end = 0
    if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x_end, y_end), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
		# only proceed if the radius meets a minimum size
		if radius > 25:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x_end), int(y_end)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
			print("The End is at " + str(x_end) + " , " + str(y_end) + "\n")
    if len(cnts_blue) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts_blue, key=cv2.contourArea)
		((x_robot, y_robot), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center_blue = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
		# only proceed if the radius meets a minimum size
		if radius > 25:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x_robot), int(y_robot)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
 			print("The Robot is at " + str(x_robot) + " , " + str(y_robot) + "\n\n")
    if abs(x_robot - x_end) < 50 and abs(y_robot - y_end) < 50:
 		print("Robot has reached the end")
    #try:
    	#for xx in range(0,len(keypoints)):
    		#x=keypoints[xx].pt[0]
    		#y=keypoints[xx].pt[1]
    		#px = gray[x,y]
    		#print("x: " + str(keypoints[xx].pt[0]) + "y: " + str(keypoints[xx].pt[1]) + "C: " + str(px))
    	#print("\n")
    #except (IndexError):
	#	pass

	# Draw detected blobs as red circles.
	#im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # Display the resulting frame
    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    cv2.imshow('keys',frame)
    #cv2.imshow('frame',im_with_keypoints)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
