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
from ros_pololu_servo.msg import Objects

from ros_pololu_servo.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint

Max_right_limit = 3
max_ir_value = 0
Max_left_limit = -3
IR_Motor_scaling = 1 #formula that maps IR range to angle range (3/IR_range)




class IR_Subscriber:

    def __init__(self):
	self.side_ir_delta_pulse = 0
	self.ir_samples = [[],[]] # 0: Front, 1: Side
	self.ir_sample_count = 0
	self.closeDistance = 50; #Distance(in centi mts.) in bad region of IR pulse graph. Meaning we are very close to wall.
	self.sideWallDistThres = 150; #Distance in centi mts.
	self.turningRight = False;

    def computeDistance(self, pulse, isFrontIR):
	index = 0 if isFrontIR else 1; # 0 for front, 1 for side
	slope = abs((pulse - ir_samples[index][ir_sample_count-4])/4);
	if slope > 8.0 or pulse > 160:
		#In bad region
		return self.closeDistance
	else: 
		return (0.00000143942*pow(pulse^5) - 0.00092177317 * pow(pulse^4) +  0.23364414584 * pow(pulse^3)  - 29.23709431046 * pow(pulse^2) + 1798.91244546323*pulse  - 43116.42420244727)

    def callback(self,irvalue):
	frontIRvalueCaptured = 0
	sideIRvalueCaptured = 0
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

	if self.ir_sample_count > 10:
		self.ir_samples[1][self.ir_sample_count -1] = np.median(np.array(self.ir_samples[1])[-10:]) #For Front IR sensor
		self.ir_samples[0][self.ir_sample_count -1] = np.median(np.array(self.ir_samples[0])[-10:]) #For Side IR sensor
		#self.plotlist.append(self.side_ir_pulse)
		

	frontDistance = computeDistance(self.ir_samples[0][self.ir_sample_count-1], True)
	sideDistance = computeDistance(self.ir_samples[1][self.ir_sample_count-1], False)

	self.CalculateDeltaPulse()
	goal = pololu_trajectoryGoal()
	traj=goal.joint_trajectory
	traj.header.stamp=rospy.Time.now()
    	traj.joint_names.append(names[0])
    	pts=JointTrajectoryPoint()
    	pts.time_from_start=rospy.Duration(1.0)

	if frontDistance > 300:		# No wall in front
		if self.turningRight:	#Now there is no wall in front. So, set "turningRight" to False.
			self.turningRight = False;
		if sideDistance == 50: # Too close to the wall
			moveMotorLeft(Max_left_limit,pts) # stear with max left angle (should be changed with function)
		elif sideDistance > 250: # Too far from the wall
			moveMotorRight(Max_right_limit,pts) # stear with max left angle (should be changed with function)
		elif sideDistance > 160:
	    		moveMotorRight(self.side_ir_delta_pulse,pts)
		elif sideDistance < 150:
			moveMotorLeft(self.side_ir_delta_pulse,pts)
		else:
			moveMotorCenter(pts)
	else:
		if frontDistance < 200:
			if self.turningRight:
				moveMotorRight(Max_right_limit,pts);
			else:
				moveMotorLeft(Max_left_limit,pts);				
		elif sideDistance == 50: # Too close to the wall
			moveMotorLeft(Max_left_limit,pts) # stear with max left angle (should be changed with function)
		elif sideDistance > 300: #If no wall is detected by right side sensor, then take right turn
			self.turningRight = True;
	    		moveMotorRight(Max_right_limit,pts)
		elif sideDistance > 160:
	    		moveMotorRight(self.side_ir_delta_pulse,pts)
		elif sideDistance < 150:
			moveMotorLeft(self.side_ir_delta_pulse,pts)
		else:
			moveMotorCenter(pts)
	
    	pts.velocities.append(1.0)
    	traj.points.append(pts)
    	client.send_goal(goal)

	#if self.ir_sample_count == 1000:
	#	print len(self.plotlist)
	#	fig, ax =plt.subplots()
	#	plt.subplot(1,2,1),plt.plot(range(1,self.ir_sample_count+1),self.ir_samples[1],label="raw")
	#	plt.subplot(1,2,2),plt.plot(range(1,len(self.plotlist)+1),self.plotlist,label="median")
	#	ax.set_axisbelow(True)
	#	ax.set_ylim(80,200)
	#	plt.legend(loc="center right")
	#	plt.show()

	if ir_sample_count > 100:
		temp = np.arry(self.ir_samples[1])[-11:];
		del ir_samples[1][:];
		self.ir_samples[1] = temp;
		temp = np.arry(self.ir_samples[0])[-11:];
		del ir_samples[0][:];
		self.ir_samples[0] = temp;
		self.ir_sample_count = 10
		
    	client.wait_for_result(rospy.Duration.from_sec(0.005))

    
    def listener(self):  
        self.sb = rospy.Subscriber("/pololu/motor_states", MotorStateList, self.callback);
	rospy.spin();

    def CalculateDeltaPulse(self):
	self.side_ir_delta_pulse = self.side_ir_lastpulse - self.side_ir_pulse[len(self.side_ir_pulse)-1]
	self.side_ir_lastpulse = self.side_ir_pulse[len(self.side_ir_pulse)-1]

def moveMotorCenter(pts):
	print "Keep servo in center position."

def moveMotorRight(distance,pts):
	print "Move right"
    	if distance >= Max_right_limit:
		pts.positions.append(-3)
    	else:
    		pts.positions.append(-(distance*IR_Motor_scaling))

def moveMotorLeft(distance,pts):
	print "Move left"
    	if distance >= Max_left_limit:
		pts.positions.append(3)
    	else:
    		pts.positions.append(distance*IR_Motor_scaling)

names=['motor_one']

if __name__ == '__main__':
    rospy.init_node('pololu_action_example_client')
    client = actionlib.SimpleActionClient('pololu_trajectory_action_server', pololu_trajectoryAction)
    IR_info = IR_Subscriber()
    client.wait_for_server()
    IR_info.listener()

