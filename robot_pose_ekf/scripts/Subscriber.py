#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.mlab as mlab
from matplotlib import pyplot as plt
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

class Subscriber:

    def __init__(self):
        self.roll = [];
        self.pitch = [];
        self.yaw = [];
        self.timeStamp = [];
	self.i = 0;
        self.sb = None;

    def callback(self, p):
        quaternion = (
            p.pose.pose.orientation.x,
            p.pose.pose.orientation.y,
            p.pose.pose.orientation.z,
            p.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.roll.append(euler[0]);
        self.pitch.append(euler[1]);
        self.yaw.append(euler[2]);
	t = p.header.stamp.secs + pow(10,-9)*p.header.stamp.nsecs;
        self.timeStamp.append(self.i);
	self.i = self.i + 1;
        
        rospy.loginfo(p)
        if len(self.timeStamp) > 1300:
            self.sb.unregister();
            rospy.signal_shutdown("Got 1300 messages");
    
    def listener(self):
        
        rospy.init_node('listener', anonymous=True)
    
        self.sb = rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.callback);
            
        rospy.spin();
        
        #print self.timeStamp;
        print "Terminated Spin in Listener";

if __name__ == '__main__':
    subscriber = Subscriber();
    subscriber.listener();        
    
    fig, ax = plt.subplots()
    plt.plot(subscriber.timeStamp, subscriber.roll, label='roll');
    plt.plot(subscriber.timeStamp, subscriber.pitch, label='pitch');
    plt.plot(subscriber.timeStamp, subscriber.yaw, label='yaw');
       
    
    ax.set_axisbelow(True)
    plt.legend(loc='center right')
    ax.set_xlabel('Time Difference(in micro seconds)')
    ax.set_ylabel('Angles')
    ax.set_title(r"Euler Angle's Plot")
    plt.show();
    
    print "Terminated Spin in Main";
