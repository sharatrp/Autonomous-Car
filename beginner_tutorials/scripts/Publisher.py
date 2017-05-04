#!/usr/bin/env python
# license removed for brevity
import rospy
import os
from std_msgs.msg import String
from beginner_tutorials.msg import Num

class Publisher:
	def talker(self):
	    pub = rospy.Publisher('PubSub1', Num, queue_size=1)
	    rospy.init_node('sender', anonymous=True)
	    rate = rospy.Rate(10) # 10hz
	    while not rospy.is_shutdown():
		num = Num();
		num.time = rospy.get_time();
		rospy.loginfo(num.time)
		pub.publish(num)
		rate.sleep()

if __name__ == '__main__':
    print os.getcwd();
    try:
	publisher = Publisher();
        publisher.talker();
    except rospy.ROSInterruptException:
        pass
