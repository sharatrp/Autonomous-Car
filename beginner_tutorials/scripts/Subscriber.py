#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.mlab as mlab
from matplotlib import pyplot as plt
from std_msgs.msg import String
from beginner_tutorials.msg import Num

class Subscriber:

    def __init__(self):
	self.time_diff = [];
	self.sb = None;

    def callback(self, num):
	self.time_diff.append(round((rospy.get_time() - num.time)*10000000));
	rospy.loginfo(num.time)
	if len(self.time_diff) > 300:
	    self.sb.unregister();
	    rospy.signal_shutdown("Got 10 messages");
    
    def listener(self):
	    
	rospy.init_node('listener', anonymous=True)

	self.sb = rospy.Subscriber("PubSub1", Num, self.callback);
	    
	rospy.spin();
	
	print self.time_diff;
	print "Terminated Spin in Listener";

if __name__ == '__main__':
    subscriber = Subscriber();
    subscriber.listener();
    
    num_bins = 50
	
    fig, ax = plt.subplots()
    n, bins, patches = plt.hist(subscriber.time_diff, bins='auto');
	
    # add a 'best fit' line
    #y = mlab.normpdf(bins, mu, sigma)
    #ax.plot(bins, y, '--')
    ax.set_xlabel('Time Difference(in micro seconds)')
    ax.set_ylabel('Frequency')
    ax.set_title(r'Histogram of Time Lapse in communication between Publisher and Subscriber')
    plt.show();
	
    print "Terminated Spin in Main";
