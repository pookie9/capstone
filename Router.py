# Goutzounis
# find the best route for the robot to the target

#Input: 
#    2D array of driveability of each pixel
#    position of robot within that 2D array, given as tuple (x,y,angle) angle is in radians
#    position of target within that 2D array, (x,y)
#Output: 
#    route to take to reach the target, or decide the target is unreachable by returning False. Note, robot will never be "on the target"
#    The format for the output is going to be a length two tuple, the first element is going to be the total distance of the route, the second element will be a list of coordinates to go to in succession in a straight line

#    This should be super easy to get working at a basic level, however there are many interesting ways we can expand. Maybe it will decide everything above .5 is driveable and everything below is not driveable and just run djkstras on that. Or maybe it will decide to do most likely to succeed path, e.g. if there are a lot of .50001s in a row you might want to avoid that area. This could be done by computing products (sum of logs really) or some other similar method.
def getRoute(drivable,robotPos,targetPos):
    distance=100
    points=[(1,1),(2,2),targetPos]
    return (distance,points)
