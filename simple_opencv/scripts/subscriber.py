#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


class Subscriber:

    def __init__(self):
	self.cnt = 0;
	self.sb = None;

    def callback(self, image):
	
	bridge = CvBridge();
	frame = bridge.imgmsg_to_cv2(image,"bgr8");
	data = np.array(frame, dtype=np.uint8);
	cv2.imshow('Image Window',data);
	imageName = "Image_"+str(self.cnt)+".jpg";
	cv2.imwrite(imageName,data);
	cv2.waitKey(3);	
	self.cnt +=1;
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
