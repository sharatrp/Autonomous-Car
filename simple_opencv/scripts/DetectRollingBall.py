#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import PIL
#import pytesseract
import matplotlib.pyplot as plt
from simple_opencv.msg import Objects

class ShapeDetector:
	def __init__(self):
		pass
 
	def is_octagon_or_circle(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)

		# if the shape is a triangle, it will have 3 vertices
		
		if len(approx) >= 8:	#Octagon
			return (1,len(approx));		
		else:			#Irrelevant contour
			return (-1,len(approx));
 

class Subscriber:

    def __init__(self):
	self.cnt = 0;
	self.sb = None;
	self.sd = ShapeDetector();
	self.orb = cv2.ORB_create()
	self.pub = None;
	self.stop_sign_check=0;
	self.count_misssed = 0;
	#im = cv2.imread("src/simple_opencv/scripts/stop.jpg");
	#cv2.imshow('',im);
	#cv2.waitKey(2);	
	self.kp2, self.des2 = self.orb.detectAndCompute(cv2.imread("src/simple_opencv/scripts/stop.jpg"),None)

    def callback(self, image):
	
	imageObj = Objects();
	imageObj.object = ""
	imageObj.area = 0.0;
		
	stopsign_distance_min  = 2000  # 4 meters
	stopsign_distance_max =  5000
	bridge = CvBridge();
	frame = bridge.imgmsg_to_cv2(image,"bgr8");
	im = frame;
	imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);
	#ret,thresh = cv2.threshold(imgray,127,255,0)
	threshold_rc,thresh = cv2.threshold(imgray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)  

	im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	#cv2.drawContours(im, contours, -1, (0,255,0), 3)			

	for shape in contours:
		area = cv2.contourArea(shape)
		(detectedShape, sides) = self.sd.is_octagon_or_circle(shape);
		if area > stopsign_distance_min and detectedShape == 1:
			
			self.count_misssed = 0
			(x, y, w, h) = cv2.boundingRect(shape);			
			newImg = im2[y-10:y+h+10, x-10:x+w+10]		
			
			(cx,cy) = (x+w/2, y+h/2);
			
			min_y = min(shape[:,0,1]);
			min_x = min(shape[:,0,0]);

			im3 = newImg.copy();                
			cv2.drawContours(im3, [shape-[min_x,min_y]], -1, (150), -1)                                          

			(i,j) = np.where(im3 == 150);                			
			ratio = 0.0;
			try:
				bins = np.bincount(newImg[i,j]);
				max_bin = bins[0] if bins[0]>bins[255] else bins[255];
				ratio = max_bin/((bins[0]+bins[255])*1.0);                
				#print "Black pixels: ",bins[0], " White pixels: ",bins[255] ," Ratio: ", ratio;
			except Exception as e:
				print "error", e.args, e;

			kp1, des1 = self.orb.detectAndCompute(newImg,None)			

			# create BFMatcher object
			bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True) 
			
			# Match descriptors.
			matches = bf.match(des1,self.des2) 			
			
			good = len(matches);

			if ratio < 0.55:
				cv2.drawContours(im, [shape], -1, (0,255,0), 3)
				#print "====================================================================================="
				#print area, good, sides, ratio	
				#print"Stop detected"
				#print "area-->",area,"ratio-->",ratio			
				#cv2.imwrite("matched_"+str(self.cnt)+"_"+str(good)+".jpg",newImg);
				#self.cnt +=1;				
				if(self.stop_sign_check > 1):
					self.stop_sign_check=self.stop_sign_check - 1

			else:
				cv2.drawContours(im, [shape], -1, (0,255,0), 3)
				print "====================================================================================="
				print "Circle detected"
				print "area--> ",area,"ratio-->",ratio
				self.stop_sign_check = self.stop_sign_check + 1
				print self.stop_sign_check
				if(self.stop_sign_check == 5):
					print "====================================================================================="
					print "====================================================================================="
					print "====================================================================================="
					print "Rolling Ball Detected"
					print "====================================================================================="
					print "====================================================================================="
					print "====================================================================================="
					self.stop_sign_check =0 

				imageObj.object = "Rolling Ball"
				imageObj.area = area;
				imageObj.cx = cx;
				imageObj.cy = cy;
		else:
			if self.stop_sign_check > 1 :	
				self.count_misssed = self.count_misssed+1
				#print self.count_misssed
				if self.count_misssed == 10000 :
					self.count_misssed = 0
					self.stop_sign_check  = self.stop_sign_check - 1	

	#cv2.drawContours(im, contours, -1, (0,255,0), 3)
	
	#Publish detected Object
	if imageObj.object == "":
		imageObj.object = "None"
	imageObj.time = rospy.get_time();
	self.pub.publish(imageObj);

	cv2.imshow('Image Window',im);
	cv2.waitKey(2);	
	#self.cnt +=1;
	#if len(self.cnt) > 4:
	#    self.sb.unregister();
	#    rospy.signal_shutdown("Got 4 messages");
    
    def listener(self):
	    
	rospy.init_node('ObjectsDetection', anonymous=True)

	self.sb = rospy.Subscriber("camera/image_raw", Image, self.callback);
	self.pub = rospy.Publisher('detectedObjects', Objects, queue_size=1)
	rate = rospy.Rate(10) # 10hz
	  
	rospy.spin();	
	print "Terminated Spin in Listener";

if __name__ == '__main__':
    subscriber = Subscriber();
    subscriber.listener();

    cv2.destroyAllWindows()
    print "Terminated Spin in Main";
