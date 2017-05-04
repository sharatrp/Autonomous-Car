#!/usr/bin/env python
import sys
import rospy
import numpy as np
import matplotlib.mlab as mlab
from matplotlib import pyplot as plt
from beginner_tutorials.srv import *

class Client:
    
    def __init__(self):
        self.timeDiff = [];
    
    def return_timestamp_client(self):
        rospy.init_node('client');
        i = 0;
        while (i<310):
            rospy.wait_for_service('return_timestamp')
            try:
                return_timestamp = rospy.ServiceProxy('return_timestamp', ReturnTime)
                resp1 = return_timestamp()
                self.timeDiff.append(round((rospy.get_time() - resp1.time)*1000000,2));
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            i+=1;

if __name__ == "__main__":    
    client = Client();
    client.return_timestamp_client();    
    print client.timeDiff;
	
    fig, ax = plt.subplots()
    plt.hist(client.timeDiff);
	
    #ax.plot(bins, y, '--')
    ax.set_xlabel('Time Difference(micro seconds)')
    ax.set_ylabel('Frequency')
    ax.set_title(r'Histogram of Time Lapse in communication between Server and Client')
    plt.show();
