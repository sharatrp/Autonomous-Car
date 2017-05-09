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

# __author__ = 'mandeep'

import rospy
import actionlib

from ros_pololu_servo.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint

k=1;

names=['Servo_Motor']


if __name__ == '__main__':
	rospy.init_node('pololu_action_example_client')
	client = actionlib.SimpleActionClient('pololu_trajectory_action_server', pololu_trajectoryAction)
	client.wait_for_server()

	# goal = pololu_trajectoryGoal()
	# traj=goal.joint_trajectory
	# traj.header.stamp=rospy.Time.now()
	# traj.joint_names.append(names[0])
	# traj.joint_names.append(names[1])

	# n = 50
	# for i in range (0,n):

	#     if(i<n/2+1):
	#         l = -0.7
	#         print l
	#     else:
	#         l = 0.7-((1.4)/(n/2))*0.5*(i-n/2)
	#         print l
	#     pts=JointTrajectoryPoint()
	#     pts.time_from_start=rospy.Duration(0.1+i*1)
	#     pts.positions.append(1.4)
	#     pts.positions.append(l)
	#     pts.velocities.append(1.0)
	#     pts.velocities.append(1.0)
	#     traj.points.append(pts)
	# i=i+1;
	# pts=JointTrajectoryPoint()
	# pts.time_from_start=rospy.Duration(i*0.1+1)
	# pts.positions.append(1.4)
	# pts.positions.append(0)
	# pts.velocities.append(1.0)
	# pts.velocities.append(1.0)
	# traj.points.append(pts)

	# # Fill in the goal here
	# client.send_goal(goal)
	# client.wait_for_result(rospy.Duration.from_sec(5.0))

	i=1;
	values = -6;
	h = 1.0;
	goal = pololu_trajectoryGoal()
	traj=goal.joint_trajectory
	traj.header.stamp=rospy.Time.now()
	traj.joint_names.append(names[0])
	#traj.joint_names.append(names[1])
	
	while values<=27:	    
		# if(k==0): #Move right
		pts=JointTrajectoryPoint()
		pts.time_from_start=rospy.Duration(i)
		pts.positions.append(values)
		pts.velocities.append(1.0)
		traj.points.append(pts)

		values = values + h;
		i +=1;
	# Fill in the goal here

	client.send_goal(goal)
	client.wait_for_result(rospy.Duration.from_sec(5.0))
	# pts=JointTrajectoryPoint()
	# pts.time_from_start=rospy.Duration(2.0)
	# pts.positions.append(-0.77)
	# pts.positions.append(0.77)
	# pts.velocities.append(1.0)
	# pts.velocities.append(1.0)
	# traj.points.append(pts)

	# pts=JointTrajectoryPoint()
	# pts.time_from_start=rospy.Duration(3.0)
	# pts.positions.append(0.77)
	# pts.positions.append(-0.77)
	# pts.velocities.append(1.0)
	# pts.velocities.append(1.0)
	# traj.points.append(pts)

	# n = 10
	# for i in range (0,n):

	#     # if(i<n/2+1):
	#     l = -0.7
	#     #     print l
	#     # else:
	#     #     l = 0.7-((1.4)/(n/2))*0.5*(i-n/2)
	#     #     print l
	#     pts=JointTrajectoryPoint()
	#     # if(i<n/4+1):
	#     pts.time_from_start=rospy.Duration(2 + i)
	#     # else if(i<n/2+1):
	#     #     pts.time_from_start=rospy.Duration(0.1+0.025*i)
	#     # else if(i<3*n/4+1):
	#     #     pts.time_from_start=rospy.Duration(0.1+0.025*i)
	#     # else:
	#     #     pts.time_from_start=rospy.Duration(0.1+0.1*i)
	#     # pts.positions.append(1.0)
	#     pts.positions.append(l)
	#     pts.velocities.append(1.0)
	#     pts.velocities.append(1.0)
	#     traj.points.append(pts)
	# pts=JointTrajectoryPoint()
	# pts.time_from_start=rospy.Duration(0.1+0.05*(i+1))
	# pts.positions.append(1.3)
	# pts.positions.append(l)
	# pts.velocities.append(1.0)
	# pts.velocities.append(1.0)
	# traj.points.append(pts)


