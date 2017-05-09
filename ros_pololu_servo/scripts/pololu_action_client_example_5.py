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
SERVO_WHEEL_ALIGNMENT_OFFSET = -5.0;	# Servo is always aligned to left. So to bring it to center, move it to -5

MIN_SPEED = 0.41;
MAX_SPEED = 0.38;

Max_right_limit = 230
Max_left_limit = 50
Max_side_threshold_dist = 200	#Max Right Side distance post which we have to steer right, meaning we are far from the wall and need to move close
Min_side_threshold_dist = 160	#Min Right Side distance post which we have to steer left, meaning we are close to the wall and need to move away
INFINITY_DIST = 300
Min_front_threshold_dist = 280
Threshold_Time_For_Rolling_Ball = 3 #For 3secs, follow different trajectoryand then come back to center of corridor.
IR_Motor_scaling = SERVO_ROTATION_RANGE/(Max_side_threshold_dist*1.0) #formula that maps IR range to angle range (25/IR_range)

names=['Servo_Motor', 'DC_Motor']

def setThresholdValuesForRollingBall():
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
	self._time = 0.0;

    def im_callback(self, obj):
	self._detectedObject = obj.object
	self._objectArea = obj.area;
	self._time = obj.time;
	print "Object Detected: ", self._detectedObject," Area: ", self._objectArea, "Time: ",self._time

    def getDetectedObject(self):
	return (self._detectedObject, self._objectArea, self._time)

class IR_Subscriber:

    def __init__(self):
	self.ir_samples = [[],[]] # 0: Front, 1: Side
	self.ir_filteredvalues = [[100 for i in range(0,MEDIAN_VALUES+1)],[100 for i in range(0,MEDIAN_VALUES+1)]]
	self.ir_sample_count = 0
	self.closeDistance = 50; #Distance(in centi mts.) in bad region of IR pulse graph. Meaning we are very close to wall.
	self.sideWallDistThres = 150; #Distance in centi mts.
	self._frontDistance = 0.0
	self._sideDistance = 0.0
	self._IRTime = 0.0
	self._LastfrontDistance = 301.0
	self._LastsideDistance = 150.0
	self._lastFrontSlope = 0.0;
	self._lastSideSlope = 0.0;

    def computeDistance(self, pulse, isFrontIR):
	index = 0 if isFrontIR else 1; # 0 for front, 1 for side

	if isFrontIR and pulse < 95:	
		return (self._LastfrontDistance if self._lastFrontSlope > 8.0 else INFINITY_DIST);	# For pulse =98, distance computed is 277.7cm		
	
	if not isFrontIR and pulse < 90:	
		return (self._LastsideDistance if self._lastSideSlope > 8.0 else INFINITY_DIST);	# For pulse =98, distance computed is 277.7cm		

	pulse = 90 if pulse < 90 else pulse
	pulse = 180 if pulse > 180 else pulse
	slope = abs((pulse - self.ir_filteredvalues[index][self.ir_sample_count-4])/4);

	if isFrontIR:
		self._lastFrontSlope = slope;
	else:
		self._lastSideSlope = slope;

	#slope2 = abs((pulse - self.ir_filteredvalues[index][self.ir_sample_count-3])/3);
	if slope > 8.0 or pulse > 160:
		#In bad region
		print "Slope: ",slope#, "Slope2: ",slope2
		return self.closeDistance
	else: 
		dist = (0.00000143942*pow(pulse,5) - 0.00092177317 * pow(pulse,4) +  0.23364414584 * pow(pulse,3)  - 29.23709431046 * pow(pulse,2) + 1798.91244546323*pulse  - 43116.42420244727)
		#if not isFrontIR and abs(dist - self._LastsideDistance) <= 5:
		#	dist = self._LastsideDistance
		return dist;

    def IR_callback(self,irvalue):
	global VALUES_UPDATED;	
	VALUES_UPDATED = False;
	frontIRvalueCaptured = 0
	sideIRvalueCaptured = 0
	self._frontDistance = self._LastfrontDistance
	self._sideDistance = self._LastsideDistance
	
	#print irvalue

	for i in irvalue.motor_states:
		if i.name == "Side_IR":
			self.ir_samples[1].append(i.pulse)
			#print self.ir_sample_count," side ir : ",i.pulse
			sideIRvalueCaptured = 1			
		if i.name == "Front_IR":
			self.ir_samples[0].append(i.pulse)
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

		self._frontDistance = self.computeDistance(self.ir_filteredvalues[0][self.ir_sample_count-1], True)
		self._sideDistance = self.computeDistance(self.ir_filteredvalues[1][self.ir_sample_count-1], False)
		#print "fd =", self._frontDistance , "sd ",self._sideDistance #, " filtered =" , self.ir_filteredvalues[1][self.ir_sample_count-1]

		self._LastfrontDistance = self._frontDistance
		self._LastsideDistance = self._sideDistance

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
	return (self._frontDistance, self._sideDistance, self._lastFrontSlope, self._lastSideSlope, self._IRTime)	


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
	self._lastAngle = SERVO_WHEEL_ALIGNMENT_OFFSET*-1.0
	self._centerTime = 0.0
	self._lastServoMovement = ""
	self._speedOffset = 0.0;
	self._probablyTurning = False;

   def move(self):
	client = actionlib.SimpleActionClient('pololu_trajectory_action_server', pololu_trajectoryAction)
	client.wait_for_server()
	self._image_subs = rospy.Subscriber("detectedObjects", Objects, self.im_callback);
	self._sb = rospy.Subscriber("/pololu/motor_states", MotorStateList, self.IR_callback);	

	while(not rospy.is_shutdown()):		
		(objectDetected, area, imTime) = ImageObjects.getDetectedObject(self);
		(frontDistance, sideDistance, frontSlope, sideSlope, iRTime) = IR_Subscriber.getIRDistances(self);
		
		if self._lastIRTime == iRTime: #or self._lastImageTime == imTime:
			continue;
		else:
			self._lastImageTime = imTime;
			self._lastIRTime = iRTime;
		
		deltaDistance = frontDistance, sideDistance
		print "frontDistance: ", frontDistance," sideDistance: ",sideDistance," _lastIRTime: ",self._lastIRTime #, objectDetected," ", area, " _lastImageTime:", self._lastImageTime, 		

		currTime = rospy.get_time()

		goal = pololu_trajectoryGoal()
		traj=goal.joint_trajectory
		traj.header.stamp = rospy.Time.now()
	    	traj.joint_names.append(names[0])
	    	pts=JointTrajectoryPoint()
	    	pts.time_from_start=rospy.Duration(0)

		if objectDetected != "None" and not self._rollingBallTrajectoryActive:
			if objectDetected == "Rolling_ball":
				print "Detected Rolling Ball."
				self.initiateTrajectoryToAvoidObstacle(self._lastIRTime);
			else:
				print "Stop sign."
				self.stop(pts);
		elif self._rollingBallTrajectoryActive:
			if (self._lastIRTime.secs - self._rollingBallTrajectoryStartTime) > 3:
				self._rollingBallTrajectoryActive = False;
				setOriginalThresholdValues();
			
		self._probablyTurning = False;
		if frontDistance >= INFINITY_DIST:		# No wall in front
			if self._turningRight or self._turningLeft:	#Now there is no wall in front. So, set "turningRight" to False.				
				self._turningRight = False;	
				self._turningLeft = False;		
			if (currTime - self._lastTurningTime) <= 5.0 and sideDistance >= INFINITY_DIST:
				sideDistance = Min_side_threshold_dist-1.0;
			if sideDistance > Max_side_threshold_dist:
		    		self.moveMotorRight(sideDistance,pts)
			elif sideDistance < Min_side_threshold_dist:
				self.moveMotorLeft(sideDistance, pts)
			else:
				self.moveMotorCenter(pts, sideSlope, currTime)
		else:
			self._probablyTurning = True;
			if frontDistance < Min_front_threshold_dist: #and (currTime - self._lastTurningTime) > 3.0:				
				if self._turningRight:
					self.moveMotorRight(Max_right_limit,pts);
				else:
					self.moveMotorLeft(Max_left_limit,pts);	
					self._turningLeft = True;
				self._lastTurningTime = currTime; #Save turning time		
			elif sideDistance > Max_side_threshold_dist:
				if sideDistance >= INFINITY_DIST: #and (currTime - self._lastTurningTime) > 3.0: #If no wall is detected by right side sensor, then take right turn
					self._turningRight = True;	
					self._lastTurningTime = currTime; #Save turning time				
		    		self.moveMotorRight(sideDistance, pts)
			elif sideDistance < Min_side_threshold_dist:
				self.moveMotorLeft(sideDistance, pts)
			else:
				self.moveMotorCenter(pts, sideSlope, currTime)
			
			
	    	pts.velocities.append(1.0)
		print "frontDistance: ", frontDistance," sideDistance: ",sideDistance, " _probablyTurning: ", self._probablyTurning, " speedOffset: ",self._speedOffset, " _lastIRTime: ",self._lastIRTime
		traj.joint_names.append(names[1]) #DC Motor
		
		speed = 0.0;
		if self._probablyTurning:
			speed = 0.46 - self._speedOffset;
			self._speedOffset += 0.003
		else:	
			speed = MAX_SPEED + ((MIN_SPEED - MAX_SPEED)*self._speedOffset/100.0)
			self._speedOffset = 0.0
		
		speed = MAX_SPEED if speed < MAX_SPEED else speed; 
		print "DC Motor Speed: ", speed
		pts.positions.append(speed);
		pts.velocities.append(1.0)
		
		print "======================================================================="

	    	traj.points.append(pts)

	    	client.send_goal(goal)

		client.wait_for_result(rospy.Duration.from_sec(0.001))		

   def initiateTrajectoryToAvoidObstacle(self, iRTime):
	setThresholdValuesForRollingBall(); 
	self._rollingBallTrajectoryActive = True;
	self._rollingBallTrajectoryStartTime = iRTime.secs;

   def stop(self, pts):
	print "do nothing"
	pts.positions.append(DC_STOP);

   def moveMotorCenter(self, pts, sideSlope, currTime):

	angle = 0.0;
	if self._lastServoMovement == "left":
		if self._lastAngle < 0:	
			angle =   SERVO_WHEEL_ALIGNMENT_OFFSET - self._lastAngle - 6;
		else:
			angle =  -1*self._lastAngle + SERVO_WHEEL_ALIGNMENT_OFFSET - 3;
	elif self._lastServoMovement == "right":
		angle = -1*self._lastAngle + SERVO_WHEEL_ALIGNMENT_OFFSET + 3; #This angle is +ve as bot was moving right. So, decreasing value by offset before giving left move instruction
	else:
		angle = SERVO_WHEEL_ALIGNMENT_OFFSET;
	
	if self._centerTime == 0.0:
		self._centerTime = currTime	
	elif (currTime - self._centerTime)*100 < 50:
		angle = angle;
	elif (currTime - self._centerTime)*100 > 50 and (currTime - self._centerTime)*100 < 200:
		angle = SERVO_WHEEL_ALIGNMENT_OFFSET;
	elif (currTime - self._centerTime)*100 > 200 and sideSlope > 2.0:
		self._centerTime = currTime
	else:		
		self._centerTime = 0.0
		self._lastServoMovement = ""
		angle = SERVO_WHEEL_ALIGNMENT_OFFSET;
	if not self._probablyTurning:	
		self._speedOffset = 0.0;
	pts.positions.append(angle);	
	print "Keep servo in center position: ",angle
	

   def moveMotorRight(self, distance, pts):	
	angle = 0.0;
	self._centerTime = 0.0
	if self._turningRight or self._turningLeft:	#distance >= Max_right_limit:
		pts.positions.append(-SERVO_MAX_ROTATION)
		angle = -SERVO_MAX_ROTATION;
    	else:
		deltaDist = Min_side_threshold_dist-distance;		
		angle = (deltaDist)*IR_Motor_scaling + SERVO_WHEEL_ALIGNMENT_OFFSET;
		if angle > -4:
			angle = -4.0;
    		pts.positions.append(angle if angle >= -SERVO_MAX_ROTATION else -SERVO_MAX_ROTATION)
		self._lastAngle  = angle;
		if not self._probablyTurning:	
			self._speedOffset =  deltaDist*-1
		
	print "Move right: ",angle
	self._lastServoMovement = "right"	
	

   def moveMotorLeft(self, distance, pts):
	angle = 0.0;
	self._centerTime = 0.0
    	if self._turningRight or self._turningLeft:	#distance <= Max_left_limit:
		pts.positions.append(SERVO_MAX_ROTATION)
		angle = SERVO_MAX_ROTATION
    	else:
		deltaDist = Min_side_threshold_dist-distance; 
		angle = deltaDist*IR_Motor_scaling + SERVO_WHEEL_ALIGNMENT_OFFSET;
		#if angle < 1:
		#	angle = 1.0
    		pts.positions.append(angle)
		self._lastAngle  = angle;
		if not self._probablyTurning:	
			self._speedOffset = deltaDist		
	print "Move left: ",angle
	self._lastServoMovement = "left"


if __name__ == '__main__':
    rospy.init_node('pololu_action_example_client')
    bot  = RobotMovement()
    bot.move(); 

