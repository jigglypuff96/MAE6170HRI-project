#!/usr/bin/env python

###################################################################################
# 
# Deanna Kocher & Jijie Zhou
# HRI Final Project
# Spring 2020
#
###################################################################################

import rospy
from geometry_msgs.msg import Twist
import geometry_msgs.msg
import numpy as np
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

class TeaFetching:
	def __init__(self):
		#create the navigation node
		rospy.init_node('navigator', anonymous = True)
		
		#create the publisher for the robot turtle
		self.robot_vel_pub = rospy.Publisher('%s/cmd_vel' % 'robot', geometry_msgs.msg.Twist, queue_size = 1)
		self.rate = rospy.Rate(100)

		#create a subscriber to keep track of the turtle pose
		self.robot_pose_sub = rospy.Subscriber('/robot/pose/', Pose, self.updatePos)
		self.pose = Pose()

		#initialize robot turtle velocity message
		self.robot_vel_msg = Twist()

		#define constants of robot motion
		self.vel_const = 0.4
		self.radius = 1.5 * pow(2, 0.5)/2

		#create the waypoints for the motion
		self.start = Pose()
		self.start.x = 4
		self.start.y = 2

		self.start_mid = Pose()
		self.start_mid.x = 4
		self.start_mid.y = 5

		self.exp = Pose()
		self.exp.x = 4
		self.exp.y = 8

		self.curve_1 = Pose()
		self.curve_1.x = 4
		self.curve_1.y = 6

		theta = np.arctan(4)
		self.curve_2 = Pose()
		self.curve_2.x = self.curve_1.x + (self.radius - self.radius * np.cos(theta))
		self.curve_2.y = self.curve_1.y - self.radius * np.sin(theta)

		self.tea_1 = Pose()
		self.tea_1.x = 8
		self.tea_1.y = 4

	def updatePos(self, data):
		self.pose = data
		self.pose.x = round(self.pose.x, 5)
		self.pose.y = round(self.pose.y, 5)


	def go_to_exp_base(self, target_1, target_2, target_3):
		#this is the standard (no timing variation) path to the experimenter
		self.robot_vel_msg.linear.x = vel_const

		targets = [target_1, target_2, target_3]

		#create a loop that continues a velocity until we reach a target
		x = 0
		while(x < len(targets)):
			dist_to_target = pow((pow(self.pose.x - targets[x].x,2) + pow(self.pose.y - targets[x].y,2)), 0.5)
			while(dist_to_target > 0.04 * (x+1)):
				#avoid div by zero
				if(rel_y[x] == 0.0):
					radius = 0
					omega = 0
				else:
					radius = (pow(rel_x[x], 2) + pow(rel_y[x], 2))/(2*rel_y[x])
					omega = vel_const/radius
				self.robot_vel_msg.angular.z = omega
				#publish
				self.robot_vel_pub.publish(self.robot_vel_msg)
				self.rate.sleep()
				print dist_to_target
				dist_to_target = pow((pow(self.pose.x - targets[x].x,2) + pow(self.pose.y - targets[x].y,2)), 0.5)
			#once a target has been reached, update the velocities to the next target
			x += 1


	def turn_around_rel(self, turn_angle):
		#turn the turtle around
		if(self.pose.theta < 0):
			target_angle = self.pose.theta + turn_angle
		elif(self.pose.theta > 0):
			target_angle = self.pose.theta - turn_angle

		error_angle = abs(self.pose.theta - target_angle)

		while(abs(error_angle > 0.01)):
			omega = self.vel_const
			self.robot_vel_msg.linear.x = 0
			self.robot_vel_msg.angular.z = omega
		 	#publish
			self.robot_vel_pub.publish(self.robot_vel_msg)
			self.rate.sleep()
			#update angle
			error_angle = abs(self.pose.theta - target_angle)	

		
			
	def go_to(self, target_1, target_2, target_3):
		#create an array of target points along route
		self.robot_vel_msg.linear.x = self.vel_const
		#set up waypoints
		targets = [target_1, target_2, target_3]
		print targets

		#create a loop that continues a velocity until we reach a target
		x = 0
		while(x < len(targets)):
			print x
			dist_to_target = pow((pow(self.pose.x - targets[x].x,2) + pow(self.pose.y - targets[x].y,2)), 0.5)

			#find the target angle with circle geometry equations (center h,k)
			x_3 = self.pose.x - targets[x].x
			y_3 = self.pose.y - targets[x].y
			print 'y_3'
			print y_3
			print 'x_3'
			print x_3
			if(x_3 != 0):
				if(x_3 > 0):
					rel_theta = np.arctan2(y_3, x_3) + np.pi
				else:
					rel_theta = np.arctan2(y_3,x_3) - np.pi
				print 'rel_theta'
				print rel_theta
				print 'turtle theta'
				print self.pose.theta
			else:
				rel_theta = 0
			angle_error = self.pose.theta - rel_theta
			if(abs(angle_error) < 0.1):
					target_angle = self.pose.theta
			elif(y_3 == 0):
				if(x_3 < 0):
					target_angle = 0
				else:
					target_angle = np.pi
			elif(x_3 == 0):
				if(y_3 < 0):
					target_angle = np.pi/2
				else:
					target_angle = -1*np.pi/2
			else:
				c_1 = pow(self.pose.x, 2) + pow(self.pose.y, 2) - pow(targets[x].x, 2) - pow(targets[x].y, 2)
				a = (pow(x_3, 2) + pow(y_3, 2))/pow(y_3, 2)
				c_2 = 1/(2 * y_3)*c_1 - self.pose.y
				b = -2 * self.pose.x - 2*c_2*x_3/y_3
				c_3 = pow(self.pose.x, 2) + pow(c_2, 2) - pow(self.radius, 2)
				print 'c_1'
				print c_1
				print 'c_2'
				print c_2
				print 'c_3'
				print c_3
				print 'a'
				print a
				print 'b'
				print b
				print 'pose x'
				print self.pose.x
				print 'pose y'
				print self.pose.y
				print 'tar x'
				print targets[x].x	
				print 'tar y'
				print targets[x].y				
				h = (-1*b + pow(pow(b,2) - 4 * a * c_3,0.5))/(2*a)
				k = -1 * x_3/y_3 * h + 1/(2*y_3)*c_1

				#angle is negative reciprocol of slope b/n target point and radius
				slope = (targets[x].y - k)/(targets[x].x - h)
				if(y_3 > 0):
					target_angle = np.arctan(-1/slope)
				else:
					target_angle = np.arctan(-1/slope) + np.pi
		
			rel_angle = self.pose.theta - target_angle
			print 'dist'
			print dist_to_target
			print 'turtle angle'
			print self.pose.theta
			print 'target_angle'
			print target_angle
			print 'rel_angle'
			print rel_angle
			#go straight forward if no angle
			if(abs(rel_angle) < 0.2):
				while(dist_to_target > 0.06 * (x+1)):
					self.robot_vel_msg.angular.z = 0
					#publish
					self.robot_vel_pub.publish(self.robot_vel_msg)
					self.rate.sleep()
					#update distances
					prev_dist = dist_to_target
					dist_to_target = pow((pow(self.pose.x - targets[x].x,2) + pow(self.pose.y - targets[x].y,2)), 0.5)
					#failsafe, for dead reckoning error
					if(dist_to_target > prev_dist):
						break
					print dist_to_target
					#once a target has been reached, update the velocities to the next target
				x += 1
			#go until angle if you need to turn
			else:
				if(targets[x].x - self.pose.x == 0 and targets[x].y - self.pose.y > 0):
					target_angle = np.pi/2
				elif(targets[x].x - self.pose.x == 0 and targets[x].y - self.pose.y < 0):
					target_angle = -1 * np.pi/2
				else:
					x_3 = self.pose.x - targets[x].x
					y_3 = self.pose.y - targets[x].y
					if(y_3 == 0):
						if(x_3 < 0):
							target_angle = 0
						else:
							target_angle = np.pi
					else:
						c_1 = pow(self.pose.x, 2) + pow(self.pose.y, 2) - pow(targets[x].x, 2) - pow(targets[x].y, 2)
						a = (pow(x_3, 2) + pow(y_3, 2))/pow(y_3, 2)
						c_2 = 1/(2 * y_3)*c_1 - self.pose.y
						b = -2 * self.pose.x - 2*c_2*x_3/y_3
						c_3 = pow(self.pose.x, 2) + pow(c_2, 2) - pow(self.radius, 2)
						h = (-1*b + pow(pow(b,2) - 4 * a * c_3,0.5))/(2*a)
						k = -1 * x_3/y_3 * h + 1/(2*y_3)*c_1

					#angle is negative reciprocol of slope b/n target point and radius
					slope = (targets[x].y - k)/(targets[x].x - h)
					if(y_3 > 0):
						target_angle = np.arctan(-1/slope)
					else:
						target_angle = np.arctan(-1/slope) + np.pi
			
				rel_angle = self.pose.theta - target_angle

				while(abs(rel_angle) > 0.005):
					if(rel_angle < 0):
						omega = self.vel_const/self.radius
					elif(rel_angle > 0):
						omega = -1 * self.vel_const/self.radius
					self.robot_vel_msg.angular.z = omega
					#publish
					self.robot_vel_pub.publish(self.robot_vel_msg)
					self.rate.sleep()
					rel_angle = self.pose.theta - target_angle
					print rel_angle
				x += 1

		#set velocities to zero when path is complete
		self.robot_vel_msg.linear.x = 0
		self.robot_vel_msg.angular.z = 0
		self.robot_vel_pub.publish(self.robot_vel_msg)
		self.rate.sleep()


if __name__ == '__main__':
	#create the navigation object
	basic_nav = TeaFetching()
	rospy.sleep(3.)
	basic_nav.go_to(basic_nav.start_mid, basic_nav.curve_1, basic_nav.exp)
	rospy.sleep(2.)
	basic_nav.turn_around_rel(np.pi)
	basic_nav.go_to(basic_nav.curve_1, basic_nav.curve_2, basic_nav.tea_1)
	rospy.sleep(2.)
	basic_nav.turn_around_rel(np.pi)
	basic_nav.go_to(basic_nav.curve_2, basic_nav.curve_1, basic_nav.exp)
	rospy.sleep(2.)
	basic_nav.turn_around_rel(np.pi)
	basic_nav.go_to(basic_nav.curve_1, basic_nav.curve_2, basic_nav.tea_1)
	rospy.sleep(2.)

