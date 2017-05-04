#!/usr/bin/env python

import rospy

import csv
import tf
from sensor_msgs.msg import Imu
from numpy import genfromtxt
import numpy
def main():
        
        
        sub=rospy.Publisher('imu_data',Imu,queue_size=10)
        rospy.init_node('Transmit')
	rate=rospy.Rate(10)
	msg = Imu()
	
        for x in range (0,100):
                
                k=numpy.zeros((9,9))
                k[0,0]=k[2,2]=0.1
                k[1,1]=0.2
                my_mag =  genfromtxt('mag.txt', delimiter=',')
                my_gyro = genfromtxt('gyro.txt', delimiter=',')
                
                my_accel =  genfromtxt('accel.txt', delimiter=',')
                
                
                
                #with open('accel.txt','r') as f:
                 #       reader = csv.reader(f)
                  ##      
                    #    your_list = map(tuple,reader)
                  
                for i in range (0,1000):
                        roll = my_mag[i,0]
                        pitch = my_mag[i,1]
                        yaw = my_mag[i,2]
                        msg.header.frame_id = "1"  
                        #if i == 1:
                        
                        #        msg.header.stamp.secs= 1200000000
                        #        msg.header.stamp.nsecs= 0
                        #else:
                
                        msg.header.stamp = rospy.Time.now()
               
                        msg.orientation.x = roll
                        msg.orientation.y = pitch
                        msg.orientation.z = yaw
                        msg.orientation.w = 0
                        msg.angular_velocity.x = my_gyro[i,0]
                        msg.angular_velocity.y =my_gyro[i,1]
                        msg.angular_velocity.z =my_gyro[i,2]
                        msg.orientation_covariance = [0.1, 0.1, 0.1,0.1,0.1,99999,99999, 99999,99999] 
                        msg.angular_velocity_covariance=[0.1,0.1, 0.1,0.1,0.1, 99999,  99999, 99999,99999] 
                        msg.linear_acceleration_covariance=[0.1, 0.1, 0.1,0.1,0.1,99999, 99999,99999,99999]  
                        
                        msg. linear_acceleration.x = my_accel[i,0]
                        msg. linear_acceleration.y =my_accel[i,1]
                        msg. linear_acceleration.z =my_accel[i,2]
                        rospy.loginfo(msg)
                        sub.publish(msg)
                        rate.sleep()
                       # for row in reader:
                           #     print(row)
                        
                
        
                
                
if __name__ == '__main__':
	#for y in range (0,5):
        main()
        
	
