#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import PIL
import pytesseract
import matplotlib.pyplot as plt

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
	#im = cv2.imread("src/simple_opencv/scripts/stop.jpg");
	#cv2.imshow('',im);
	#cv2.waitKey(2);	
	self.kp2, self.des2 = self.orb.detectAndCompute(cv2.imread("src/simple_opencv/scripts/stop.jpg"),None)

    def callback(self, image):
	
	bridge = CvBridge();
	frame = bridge.imgmsg_to_cv2(image,"bgr8");
	im = frame;
	imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY);
	ret,thresh = cv2.threshold(imgray,127,255,0)
	im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)			

	for shape in contours:
		area = cv2.contourArea(shape)
		(detectedShape, sides) = self.sd.is_octagon_or_circle(shape);
		if area > 800 and detectedShape == 1:
			cv2.drawContours(im, [shape], -1, (0,255,0), 3)
			#M = cv2.moments(shape)
			#cX = int((M["m10"] / M["m00"]))
			#cY = int((M["m01"] / M["m00"]))
			#cv2.putText(im, str(area), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			(x, y, w, h) = cv2.boundingRect(shape);
			#(x,y),radius = cv2.minEnclosingCircle(shape)
			newImg = im2[y-10:y+h+10, x-10:x+w+10]		
			
			#newImg = im2[y:y+h, x:x+w]	

			min_y = min(shape[:,0,1]);
			min_x = min(shape[:,0,0]);
			newImg2 = newImg|3;

			im3 = newImg.copy();
			im4 = newImg.copy();
			cv2.drawContours(im3, [shape-[min_x,min_y]], -1, (255), -1)
			cv2.drawContours(im4, [shape-[min_x,min_y]], -1, (1), -1)							

			#plt.subplot(221), plt.imshow(im)
			#plt.subplot(222), plt.imshow(im3);
			#plt.subplot(223), plt.imshow(im4);


			mask = im3^im4; #Image with 254 and 0 intensities

			#print mask[mask.shape[0]/2,mask.shape[1]/2];
			masked_img = cv2.bitwise_and(newImg2, mask)

			#plt.subplot(224), plt.imshow(masked_img)	#plt.imshow(im3^im4);

			hist_mask = cv2.calcHist([masked_img],[0],None, [256],[0,256]);

			#print "masked_img Shape: ",masked_img.shape;
			#print np.where(hist_mask!=0), hist_mask[2][0], hist_mask[254][0] 
			
			ratio = 0.0;
			try:
				max_bin = hist_mask[2][0] if hist_mask[2][0] > hist_mask[254][0] else hist_mask[254][0];
				ratio = max_bin/((hist_mask[254][0]+hist_mask[2][0])*1.0);
			except:
				print "Bin length: ",len(hist_mask),"\n",hist_mask

			kp1, des1 = self.orb.detectAndCompute(newImg,None)			

			# create BFMatcher object
			bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True) 
			#bf = cv2.BFMatcher()

			# Match descriptors.
			matches = bf.match(des1,self.des2) 
			#matches = bf.knnMatch(des1,self.des2, k=2)
			
			good = len(matches);
			#good = 0;
			#for m,n in matches:
			#	if n.distance < 0.75*m.distance:
			#		good +=1;

			# Sort them in the order of their distance.
			#matches = sorted(matches, key = lambda x:x.distance)
			#bins = [];
			#try:			
			#	bins = np.bincount(im2[shape[:,0,0],shape[:,0,1]]);
			#except:
			#	plt.imshow(im2);	
			#	plt.show();	
			#	cv2.imwrite("Error_"+str(self.cnt)+"_"+str(good)+".jpg",im);
			
			#ratio = 0.0;
			
			#bin_length = len(bins)-1;
			#try:
			#	max_bin = bins[0] if bins[0]>bins[bin_length] else bins[bin_length];
			#	ratio = max_bin/((bins[0]+bins[bin_length])*1.0);
			#except:
			#	print "Bin length: ",len(bins),"\n",bins

			# Draw first 10 matches.
			if good > 50 and ratio < 0.85:
				print area, good, sides, ratio
				#img3 = cv2.drawMatches(newImg,kp1,train,kp2,matches, None, flags=2)
				cv2.imwrite("matched_"+str(self.cnt)+"_"+str(good)+".jpg",newImg);
				self.cnt +=1;
			elif ratio > 0.85:
				print "Circle: ",area, good, sides, ratio

	#cv2.drawContours(im, contours, -1, (0,255,0), 3)
	cv2.imshow('Image Window',im);
	cv2.waitKey(2);	
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
