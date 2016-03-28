import cv2;
import Locator;
import time;

cap = cv2.VideoCapture(0)

while(True):
	ret, frame = cap.read()
        print "here"
        pos=Locator.getRobotPos(frame)
        cv2.imshow("blah",frame)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
"""
for i in range(30):
        ret, frame = cap.read()

ret, frame = cap.read()
cv2.imwrite('robotPic.png',frame)

cap.release()
cv2.destroyAllWindows()
"""
