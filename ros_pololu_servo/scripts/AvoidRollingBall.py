#! /usr/bin/env python
# Copyright (c) 2014, OpenCog Foundation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the OpenCog Foundation nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

__author__ = 'vibhor'

import rospy
import actionlib
import matplotlib.pyplot as plt
import numpy as np
from ros_pololu_servo.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint

DC_STOP = 0.5;

MEDIAN_VALUES = 5;

VALUES_UPDATED = False;


SERVO_MAX_ROTATION = 25.0;
SERVO_ROTATION_RANGE = 20.0; 
SERVO_WHEEL_ALIGNMENT_OFFSET = -5;	# Servo is always aligned to left. So to bring it to center, move it to -5

Max_right_limit = 230		#Distance after which we have to turn servo extreme right 
Max_left_limit = 50		#Distance below which we have to turn servo extreme left 
Max_side_threshold_dist = 170	#Max Right Side distance post which we have to steer right, meaning we are far from the wall and need to move close
Min_side_threshold_dist = 165	#Min Right Side distance post which we have to steer left, meaning we are close to the wall and need to move away
INFINITY_DIST = 280
Min_front_threshold_dist = 200
Threshold_Time_For_Rolling_Ball = 3 #For 3secs, follow different trajectoryand then come back to center of corridor.
IR_Motor_scaling = SERVO_ROTATION_RANGE/(Max_side_threshold_dist*1.0) #formula that maps IR range to angle range (25/IR_range)

names=['Servo_Motor', 'DC_Motor']

def setThresholdValuesForRightRollingBall():
	global Max_right_limit, Max_left_limit, Max_side_threshold_dist, Min_side_threshold_dist
	Max_right_limit = 300
	Max_left_limit = 180
	Max_side_threshold_dist = 260	#Max Right Side distance post which we have to steer right, meaning we are far from the wall
	Min_side_threshold_dist = 250	#Min Right Side distance post which we have to steer left, meaning we are close to the wall 

def setThresholdValuesForLeftRollingBall(): #In this case, move bot right
	global Max_right_limit, Max_left_limit, Max_side_threshold_dist, Min_side_threshold_dist
	Max_right_limit = 300
	Max_left_limit = 180
	Max_side_threshold_dist = 260	#Max Right Side distance post which we have to steer right, meaning we are far from the wall
	Min_side_threshold_dist = 250	#Min Right Side distance post which we have to steer left, meaning we are close to the wall 


def setOriginalThresholdValues():
	global Max_right_limit, Max_left_limit, Max_side_threshold_dist, Min_side_threshold_dist
	Max_right_limit = 250
	Max_left_limit = 50
	Max_side_threshold_dist = 160	#Max Right Side distance post which we have to steer right, meaning we are far from the wall
	Min_side_threshold_dist = 150	#Min Right Side distance post which we have to steer left, meaning we are close to the wall 


class ImageObjects:
    def __init__(self):        
	self._detectedObject = "None"
	self._objectArea = 0.0;
	self._cx = 0.0;
	self._cy = 0.0;
	self._time = 0.0;

    def im_callback(self, obj):
	self._detectedObject = obj.object
	self._objectArea = obj.area;
	self._cx = obj.cx;
	self._cy = obj.cy;
	self._time = obj.time;
	#print "Object Detected: ", self._detectedObject," Area: ", self._objectArea, "Time: ",self._time

    def getDetectedObject(self):
	return (self._detectedObject, self._objectArea, self._cx, self._cy, self._time)

class IR_Subscriber:

    def __init__(self):
	self.ir_samples = [[],[]] # 0: Left, 1: Right
	self.ir_filteredvalues = [[100 for i in range(0,MEDIAN_VALUES+1)],[100 for i in range(0,MEDIAN_VALUES+1)]]
	self.ir_sample_count = 0
	self.closeDistance = 50; #Distance(in centi mts.) in bad region of IR pulse graph. Meaning we are very close to wall.
	self.sideWallDistThres = 150; #Distance in centi mts.
	self._LeftsideDistance = 0.0
	self._RightsideDistance = 0.0
	self._IRTime = 0.0
	self._LastLsideDistance = 140.0
	self._LastRsideDistance = 160.0

    def computeDistance(self, pulse, isLeftIR):
	index = 0 if isLeftIR else 1; # 0 for Left, 1 for Right

	pulse = 90 if pulse < 90 else pulse
	pulse = 180 if pulse > 180 else pulse
	slope = abs((pulse - self.ir_filteredvalues[index][self.ir_sample_count-4])/4);
	#slope2 = abs((pulse - self.ir_filteredvalues[index][self.ir_sample_count-3])/3);
	if slope > 8.0 or pulse > 160:
		#In bad region
		print "Slope: ",slope#, "Slope2: ",slope2
		return self.closeDistance
	else: 
		dist = (0.00000143942*pow(pulse,5) - 0.00092177317 * pow(pulse,4) +  0.23364414584 * pow(pulse,3)  - 29.23709431046 * pow(pulse,2) + 1798.91244546323*pulse  - 43116.42420244727)
		if not isLeftIR and abs(dist - self._LastRsideDistance) <= 5:
			dist = self._LastRsideDistance
		elif not isLeftIR and abs(dist - self._LastLsideDistance) <= 5:
			dist = self._LastLsideDistance
		return dist;

    def IR_callback(self,irvalue):
	global VALUES_UPDATED;	
	VALUES_UPDATED = False;
	frontIRvalueCaptured = 0
	sideIRvalueCaptured = 0
	self._LeftsideDistance = self._LastLsideDistance
	self._RightsideDistance = self._LastRsideDistance
	
	#print irvalue

	for i in irvalue.motor_states:
		if i.name == "Side_IR":  #right 
			self.ir_samples[1].append(i.pulse)
			#print self.ir_sample_count," side ir : ",i.pulse
			sideIRvalueCaptured = 1			
		if i.name == "Front_IR": #left
			self.ir_samples[0].append(90)#i.pulse)
			#print self.ir_sample_count," front ir : ",i.pulse				
			frontIRvalueCaptured = 1
		if sideIRvalueCaptured and frontIRvalueCaptured:
			self.ir_sample_count += 1	
			break	

	if not sideIRvalueCaptured and not frontIRvalueCaptured:	
		print "No IR sensor data received"
		return;
	
	#print self.ir_sample_count, self.ir_filteredvalues[1]
	
	if self.ir_sample_count > MEDIAN_VALUES:
		#print "sample count = ", self.ir_sample_count, " sample len =" , len(self.ir_filteredvalues[1])
		self.ir_filteredvalues[1].append(np.median(np.array(self.ir_samples[1])[-MEDIAN_VALUES:])) #For Front IR sensor
		self.ir_filteredvalues[0].append(np.median(np.array(self.ir_samples[0])[-MEDIAN_VALUES:])) #For Side IR sensor
		#self.plotlist.append(self.side_ir_pulse)

		print self.ir_sample_count," side ir : ",self.ir_samples[1][self.ir_sample_count-1], 	"Median: ", self.ir_filteredvalues[1][self.ir_sample_count-1]
		print self.ir_sample_count," front ir : ",self.ir_samples[0][self.ir_sample_count-1], "Median: ", self.ir_filteredvalues[0][self.ir_sample_count-1]

		self._LeftsideDistance = self.computeDistance(self.ir_filteredvalues[0][self.ir_sample_count-1], True)
		self._RighttsideDistance = self.computeDistance(self.ir_filteredvalues[1][self.ir_sample_count-1], False)
		#print "fd =", self._frontDistance , "sd ",self._sideDistance #, " filtered =" , self.ir_filteredvalues[1][self.ir_sample_count-1]

		self._LastLsideDistance = self._LeftsideDistance
		self._LastRsideDistance = self._RightsideDistance

		self._IRTime = rospy.get_time();

		VALUES_UPDATED = True;
		
		if self.ir_sample_count == 1000:
			print "Count: ", len(self.ir_filteredvalues[0]), len(self.ir_samples[0])
			plt.plot(range(0,self.ir_sample_count+1), self.ir_filteredvalues[0],"r")
			plt.plot(range(0,self.ir_sample_count), self.ir_samples[0],"y")
			plt.show()
		
	#if self._sideDistance is None:
	#	print "Median: ", self.ir_filteredvalues[0],"\nSamples: ", self.ir_samples[0]
	
	#print "fd =", self._frontDistance , "sd ",self._sideDistance, "IRTime: ", self._IRTime

	

	if self.ir_sample_count > 500:
		temp = np.array(self.ir_samples[1])[-(MEDIAN_VALUES+2):];
		del self.ir_samples[1][:];
		self.ir_samples[1] = temp.tolist();
		temp = np.array(self.ir_samples[0])[-(MEDIAN_VALUES+2):];
		del self.ir_samples[0][:];
		self.ir_samples[0] = temp.tolist();
		temp = np.array(self.ir_filteredvalues[1])[-(MEDIAN_VALUES+2):];
		del self.ir_filteredvalues[1][:];
		self.ir_filteredvalues[1] = temp.tolist();
		temp = np.array(self.ir_filteredvalues[0])[-(MEDIAN_VALUES+2):];
		del self.ir_filteredvalues[0][:];
		self.ir_filteredvalues[0] = temp.tolist();
		self.ir_sample_count = MEDIAN_VALUES+1
	    	

    def getIRDistances(self):
	#print (self._frontDist, self._sideDist, self._IRTime)
	return (self._LeftsideDistance, self._RightsideDistance, self._IRTime)	


class RobotMovement(ImageObjects, IR_Subscriber):
   def __init__(self):
	ImageObjects.__init__(self);
	IR_Subscriber.__init__(self);
	self._turningRight = False;
	self._turningLeft = False;
	self._lastImageTime = 0.0;
	self._lastIRTime = 0.0;
	self._rollingBallTrajectoryActive = False;
	self._rollingBallTrajectoryStartTime = 0;
	self._lastTurningTime = 0.0;
	self._continuousDetections = 0;
	self._nOfMiss = 0;
	self._lastCx = 0;
	self._lastDeltaCx = [];
	self._lastBotDirection = "";

   def move(self):
	client = actionlib.SimpleActionClient('pololu_trajectory_action_server', pololu_trajectoryAction)
	client.wait_for_server()
	self._image_subs = rospy.Subscriber("detectedObjects", Objects, self.im_callback);
	self._sb = rospy.Subscriber("/pololu/motor_states", MotorStateList, self.IR_callback);	

	while(not rospy.is_shutdown()):		
		(objectDetected, area, cx, cy, imTime) = ImageObjects.getDetectedObject(self);
		(LeftsideDistance, RightsideDistance, iRTime) = IR_Subscriber.getIRDistances(self);
		
		if self._lastIRTime == iRTime: #or self._lastImageTime == imTime:
			continue;
		else:
			self._lastImageTime = imTime;
			self._lastIRTime = iRTime;
		

		print "LeftsideDistance: ", LeftsideDistance," RightsideDistance: ",RightsideDistance," _lastIRTime: ",self._lastIRTime #, objectDetected," ", area, " _lastImageTime:", self._lastImageTime, 		

		currTime = rospy.get_time()
		
		goal = pololu_trajectoryGoal()
		traj=goal.joint_trajectory
		traj.header.stamp = rospy.Time.now()
	    	traj.joint_names.append(names[0])
	    	pts=JointTrajectoryPoint()
	    	pts.time_from_start=rospy.Duration(0)
		ballDirection = ""
		sideDistance = 0.0

		if objectDetected != "None" and not self._rollingBallTrajectoryActive:
			if objectDetected == "Rolling_ball":
				print "Detected Rolling Ball."
				if self._lastCx > 0:
					self._lastCx.append(cx-self._lastCx);				
				if(len(self._lastDeltaCx) >= 5):
					ballDirection = checkListOrder();					
					self.initiateTrajectoryToAvoidObstacle(self._lastIRTime, ballDirection);
		elif self._rollingBallTrajectoryActive:
			if (self._lastIRTime.secs - self._rollingBallTrajectoryStartTime) > Threshold_Time_For_Rolling_Ball:
				self._rollingBallTrajectoryActive = False;
				setOriginalThresholdValues();
		else:
			self._nOfMiss += 1;
			if self._nOfMiss > 1:
				self._lastCx = 0.0;
				del self._lastDeltaCx[:];
				self._nOfMiss = 0;
		
		self._lastCx = cx;
		
		if self._rollingBallTrajectoryActive and ballDirection == "right":
			sideDistance = RightsideDistance
		elif self._rollingBallTrajectoryActive and ballDirection == "left":
			sideDistance = LeftsideDistance
		else:
			sideDistance = RightsideDistance
			
		
		if sideDistance > Max_side_threshold_dist:
	    		self.moveMotorRight(sideDistance,pts)
		elif sideDistance < Min_side_threshold_dist:
			self.moveMotorLeft(sideDistance, pts)
		else:
			self.moveMotorCenter(pts)
		
		print "======================================================================="
	    	pts.velocities.append(1.0)

		traj.joint_names.append(names[1]) #DC Motor
		pts.positions.append(0.38)
		pts.velocities.append(1.0)

	    	traj.points.append(pts)

	    	client.send_goal(goal)

		client.wait_for_result(rospy.Duration.from_sec(0.005))		

   def initiateTrajectoryToAvoidObstacle(self, iRTime, ballDirection):
	if ballDirection == "left":
		setThresholdValuesForRightRollingBall();
	elif ballDirection == "right":
		setThresholdValuesForLeftRollingBall();
	if ballDirection != "junk":
		self._rollingBallTrajectoryActive = True;
		self._rollingBallTrajectoryStartTime = iRTime.secs;
	else:
		del self._lastDeltaCx[:];

   def checkListOrder(self):
	pos = 0;
	neg = 0
	for i in self._lastDeltaCx:
		if i >= 0:
			pos += 1;
		else:
			neg += 1;
	if pos > neg and (pos - neg) >= 7:
		return "right"
	elif pos < neg and (neg - pos) >= 7:
		return "left"
	else:
		return "junk"

   def stop(self, pts):
	print "do nothing"
	pts.positions.append(DC_STOP);

   def moveMotorCenter(self, pts):
	print "Keep servo in center position."
	pts.positions.append(SERVO_WHEEL_ALIGNMENT_OFFSET)
	self._lastBotDirection = "center"

   def moveMotorRight(self, distance, pts):	
	val = "";    	
	angle = (Min_side_threshold_dist-distance)*IR_Motor_scaling + SERVO_WHEEL_ALIGNMENT_OFFSET;
	pts.positions.append(angle if angle >= -SERVO_MAX_ROTATION else )
	val = str(angle)
	print "Move right: ",val
	self._lastBotDirection = "right"

   def moveMotorLeft(self, distance, pts):
	val = "";    
	angle = (Min_side_threshold_dist-distance)*IR_Motor_scaling + SERVO_WHEEL_ALIGNMENT_OFFSET;
	pts.positions.append(angle)
	val = str(angle)
	print "Move left: ",val
	self._lastBotDirection = "left"


if __name__ == '__main__':
    rospy.init_node('pololu_action_example_client')
    bot  = RobotMovement()
    bot.move(); 

