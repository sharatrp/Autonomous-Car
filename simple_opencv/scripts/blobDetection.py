#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import PIL
import pytesseract

class ShapeDetector:
	def __init__(self):
		pass
 
	def is_octagon_or_circle(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)

		# if the shape is a triangle, it will have 3 vertices
		if len(approx) == 8:	#Octagon
			return 1;
		elif len(approx) > 8:	#Circle
			return 2;
		else:			#Irrelevant contour
			return -1;
 

class Subscriber:

    def __init__(self):
	self.cnt = 0;
	self.sb = None;
	self.sd = ShapeDetector();
	self.orb = cv2.ORB_create()
	self.kp2, self.des2 = self.orb.detectAndCompute(cv2.imread("src/simple_opencv/scripts/stop.jpg"),None)

    def callback(self, image):
	
	bridge = CvBridge();
	frame = bridge.imgmsg_to_cv2(image,"bgr8");
	im = frame;
	imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);	
				
	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()
	 
	# Change thresholds
	params.minThreshold = 5;
	params.maxThreshold = 250;
	 
	# Filter by Area.
	params.filterByArea = True
	params.minArea = 500
	 
	# Filter by Circularity
	params.filterByCircularity = True
	params.minCircularity = 1
	 
	# Filter by Convexity
	params.filterByConvexity = True
	params.minConvexity = 0.87
	 
	# Filter by Inertia
	params.filterByInertia = True
	params.minInertiaRatio = 0.87
	 
	# Create a detector with the parameters
	ver = (cv2.__version__).split('.')
	if int(ver[0]) < 3 :
	    detector = cv2.SimpleBlobDetector(params)
	else : 
	    detector = cv2.SimpleBlobDetector_create(params)

	#detector = cv2.SimpleBlobDetector_create()
 
	# Detect blobs.
	keypoints = detector.detect(imgray)
	 
	# Draw detected blobs as red circles.
	# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
	im_with_keypoints = cv2.drawKeypoints(imgray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	 
	# Show keypoints
	cv2.imshow("Keypoints", im_with_keypoints)
	cv2.waitKey(2)
	#self.cnt +=1;
	#if len(self.cnt) > 4:
	#    self.sb.unregister();
	#    rospy.signal_shutdown("Got 4 messages");
    
    def listener(self):
	    
	rospy.init_node('listener', anonymous=True)

	self.sb = rospy.Subscriber("camera/image_raw", Image, self.callback);
	    
	rospy.spin();	
	print "Terminated Spin in Listener";

if __name__ == '__main__':
    subscriber = Subscriber();
    subscriber.listener();

    cv2.destroyAllWindows()
    print "Terminated Spin in Main";
