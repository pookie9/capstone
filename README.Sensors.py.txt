Sensors.py contains two classes, Sensors, and SimSensors

Sensors is not started yet, it will eventually be how you get a picture, get kinect data, and move the robot


SimSensors is a simulator of a robot moving through a scenario

	   -Create a new SimSensors with s=SimSensors(mazePic, heights, initPos), mazePic is the file name of the maze, e.g.
	   "BasicMaze.png" (already exists), heights is a file name of the corresponding heights e.g. BasicMazeHeights (already exists)
	   (see note below for format of heights file)
	   and initPos is a 3 tuple of (x,y,orientation), x and y in pixels, orientation in radians, 0=straight right, increases
	   in orientation go counter-clockwise (due to upside down cartesian coordinates of pictures

	   -getPic(), how you get a picture of the maze with the robot on it, returns an opencv picture

	   -getKinectData(), returns a list of 3d points (x,y,z), (x,y) in pixels, z in height units (which are whatever we want them to be)

	   -move(newPos), moves the robot in a straight line to the newPos which is (x,y) x and y are in pixels, this needs to be redone 
	   eventually. Currently it allows the robot to move over walls, which is bad.

	   -showBot() shows the robot and his field of vision, the center of his field of vision is a red line, the sides are grey lines

	   Note, the format for the heights file for a mxn picture is an m line text file, where each line has n numbers with a 
	   comma as the delimiter. each number is the corresponding pixel's height in height units (no idea what to make these yet).
	   
