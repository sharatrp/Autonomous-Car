#!/usr/bin/env python
# license removed for brevity
#!/usr/bin/env python
# license removed for brevity
import rospy
import os
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import csv
import tf
import os



class Publisher:
    def talker(self, type_of_data):

        cwd = os.getcwd()        
        gy = [];
        accel = [];
        mag = [];
        timeStamp = [];  
        
        msg1 = Odometry();
        
	gladiator_files = ['/gyro_gladiator.txt','/mag_gladiator.txt','/accel_gladiator.txt','/timestamp_gladiator.txt']
	simluated_files = ['/gyro.txt','/mag.txt','/accel.txt','/timestamp.txt']
	files = None;
		
	if type_of_data == 1:
		files = gladiator_files;
	else:
		files = simluated_files;	
	
	print 'type_of_data: ',type_of_data, files	

	f = open(cwd+files[0],'r')  
        reader = csv.DictReader(f, fieldnames=['x','y','z'])
        for row in reader:
	    quaternion = tf.transformations.quaternion_from_euler(float(row['x'].strip()), float(row['y'].strip()), float(row['z'].strip()));
            gy.append(quaternion);
            
        f.close();
               
        f = open(cwd+files[1],'r')        
        reader = csv.DictReader(f, fieldnames=['x','y','z'])
        for row in reader:	    
            mag.append([float(row['x'].strip()), float(row['y'].strip()), float(row['z'].strip())]);
            
        f.close();
                
        f = open(cwd+files[2],'r')     
        reader = csv.DictReader(f, fieldnames=['x','y','z'])
        for row in reader:
            accel.append([float(row['x'].strip()), float(row['y'].strip()), float(row['z'].strip())]);
            

        f.close();
                
        f = open(cwd+files[3],'r')        
        reader = csv.DictReader(f, fieldnames=['x','y'])
        for row in reader:            
            timeStamp.append(float(row['y'].strip()));
            
        f.close();
        
        pub = rospy.Publisher('imu_data', Imu, queue_size=10)
        sub = rospy.Publisher('vo',Odometry,queue_size=10)
        rospy.init_node('sender', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        print "gy: ",len(gy), "accel: ",len(accel), "mag: ",len(mag), "gy: ",len(timeStamp)

        i=0;
        while not rospy.is_shutdown():            
                        
            imu = Imu();
            imu.header.stamp = rospy.Time.now();
            msg1.header.stamp = imu.header.stamp;
            #imu.header.stamp.secs =  int(timeStamp[i]);            #timestamp.txt
            #imu.header.stamp.nsecs = int((timeStamp[i] - imu.header.stamp.secs)*pow(10,9));
            imu.header.frame_id = 'base_footprint'
            #gyro.txt
            #print 'i= ',i,' gy= ',gy[i];
            #imu.angular_velocity.x = mag[i][0];
            #imu.angular_velocity.y = mag[i][1];
            #imu.angular_velocity.z = mag[i][2];
            #accel.txt
            #print ' accel= ',accel[i];
            #imu.linear_acceleration.x = accel[i][0];
            #imu.linear_acceleration.y = accel[i][1];
            #imu.linear_acceleration.z = accel[i][2];

            #mag.txt
            #print ' mag= ',mag[i];	    
	    imu.orientation.x = gy[i][0]
	    imu.orientation.y = gy[i][1]
	    imu.orientation.z = gy[i][2]
	    imu.orientation.w = gy[i][3]            

            pub.publish(imu)                        
                            
            msg1.header.frame_id = "base_footprint"        
            msg1.pose.pose.position.x = 0           
            msg1.pose.pose.position.y = 0             
            msg1.pose.pose.position.z = 0            
            msg1.pose.pose.orientation.x = 1              
            msg1.pose.pose.orientation.y = 0              
            msg1.pose.pose.orientation.z = 0              
            msg1.pose.pose.orientation.w = 0              
            msg1.pose.covariance = [0.1, 0, 0, 0, 0, 0, 
                    0, 0.1, 0, 0, 0, 0, 
                    0, 0, 0.1, 0, 0, 0,  
                    0, 0, 0, 99999, 0, 0,  
                    0, 0, 0, 0, 99999, 0,  
                    0, 0, 0, 0, 0, 99999]
            #rospy.loginfo(imu)  
            sub.publish(msg1)
            
            
            rate.sleep()
            i = i+1;            
            if i == len(gy)-1:
                break;

if __name__ == '__main__':

    type_of_data = raw_input('Choose the type pf data: 1. Gladiator  2. Synthetic ==>')    
    try:
        publisher = Publisher();
        publisher.talker(int(type_of_data));
    except rospy.ROSInterruptException:
        pass



