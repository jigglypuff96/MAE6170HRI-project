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
import math

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
		self.vel_const = 1.0
		self.radius = 1.5 * pow(2, 0.5)/2

		#create the waypoints for the motion
		self.start = Pose()
		self.start.x = 4.0
		self.start.y = 2.0

		self.exp = Pose()
		self.exp.x = 4.0
		self.exp.y = 8.0

		self.two = Pose()
		self.two.x = 4.0
		self.two.y = self.exp.y - 1.0/3.0

		self.three = Pose()
		self.three.x = 4.0 
		self.three.y = self.two.y - 1.0/3.0

		self.four = Pose()
		self.four.x = 4.0
		self.four.y = self.three.y - 1.0/3.0

		self.five = Pose()
		self.five.x = 4.0
		self.five.y = self.four.y - 1.0/3.0

		self.six = Pose()
		self.six.x = 4.0
		self.six.y = self.five.y - 1.0/3.0

		self.seven = Pose()
		self.seven.x = 4.0 
		self.seven.y = self.six.y - 1.0/3.0

		self.eight = Pose()
		self.eight.x = 4.0
		self.eight.y = self.seven.y - 1.0/3.0

		theta = np.arctan(4)
		self.nine = Pose()
		self.nine.x = self.eight.x + (self.radius - self.radius * np.cos(theta))
		self.nine.y = self.eight.y - self.radius * np.sin(theta)

		self.tea_1 = Pose()
		self.tea_1.x = 8.0
		self.tea_1.y = 4.0

		m = (self.nine.y - self.tea_1.y)/(self.nine.x - self.tea_1.x)
		b = self.tea_1.y - m*self.tea_1.x
		x_int = (self.tea_1.x - self.nine.x)/9

		self.ten = Pose()
		self.ten.x = self.nine.x + x_int
		self.ten.y = m * self.ten.x + b

		self.eleven = Pose()
		self.eleven.x = self.ten.x + x_int
		self.eleven.y = m * self.eleven.x + b

		self.twelve = Pose()
		self.twelve.x = self.eleven.x + x_int
		self.twelve.y = m * self.twelve.x + b

		self.thirteen = Pose()
		self.thirteen.x = self.twelve.x + x_int
		self.thirteen.y = m * self.thirteen.x + b
		
		self.fourteen = Pose()
		self.fourteen.x = self.thirteen.x + x_int
		self.fourteen.y = m * self.fourteen.x + b

		self.fifteen = Pose()
		self.fifteen.x = self.fourteen.x + x_int
		self.fifteen.y = m * self.fifteen.x + b

		self.sixteen = Pose()
		self.sixteen.x = self.fifteen.x + x_int
		self.sixteen.y = m * self.sixteen.x + b

		self.seventeen = Pose()
		self.seventeen.x = self.sixteen.x + x_int
		self.seventeen.y = m * self.seventeen.x + b


	def updatePos(self, data):
		self.pose = data
		self.pose.x = round(self.pose.x, 5)
		self.pose.y = round(self.pose.y, 5)

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
			rel_pos = Pose()
			rel_pos.x = targets[x].x - self.pose.x
			rel_pos.y = targets[x].y - self.pose.y
			rel_pos = self.target_rel_robot(rel_pos)
			print 'rel X'
			print rel_pos.x
			print 'rel Y'
			print rel_pos.y
			#x_3 = self.pose.x - targets[x].x
			#y_3 = self.pose.y - targets[x].y
			#target_angle = find_target_angle(x_3,  y_3)
			#rel_angle = self.pose.theta - target_angle
			print 'turtle angle'
			print self.pose.theta
			print 'turtle x'
			print self.pose.x
			print 'turtle y'
			print self.pose.y
			#print 'target_angle'
			#print target_angle
			#print 'rel_angle'
			#print rel_angle
			#go straight forward if no angle
			if(abs(rel_pos.y) < 0.3):
				while(dist_to_target > 0.015 * (x+1)):
					self.robot_vel_msg.angular.z = 0
					#publish
					self.robot_vel_pub.publish(self.robot_vel_msg)
					self.rate.sleep()
					#update distances
					prev_dist = dist_to_target
					dist_to_target = pow((pow(self.pose.x - targets[x].x,2) + pow(self.pose.y - targets[x].y,2)), 0.5)
					print 'straight'
					#failsafe, for dead reckoning error
					if(dist_to_target > prev_dist):
						break
					#once a target has been reached, update the velocities to the next target
				x += 1
			#go until angle if you need to turn
			else:
				x_3 = targets[x].x - self.pose.x
				y_3 = targets[x].y - self.pose.y
				target_angle = self.find_target_angle(x_3, y_3, targets[x])
				
				rel_angle = self.pose.theta - target_angle

				while(abs(rel_angle) > 0.006):
					if(rel_angle < 0):
						omega = self.vel_const/self.radius
					elif(rel_angle > 0):
						omega = -1 * self.vel_const/self.radius
					self.robot_vel_msg.angular.z = omega
					#publish
					self.robot_vel_pub.publish(self.robot_vel_msg)
					self.rate.sleep()
					print rel_angle
					rel_angle = self.pose.theta - target_angle
				x += 1

		#set velocities to zero when path is complete
		self.robot_vel_msg.linear.x = 0
		self.robot_vel_msg.angular.z = 0
		self.robot_vel_pub.publish(self.robot_vel_msg)
		self.rate.sleep()

	def target_rel_robot(self, target):
		rel_x = math.cos(self.pose.theta)*target.x + math.sin(self.pose.theta)*target.y
		rel_y = -1.0 * math.sin(self.pose.theta)*target.x + math.cos(self.pose.theta)*target.y
		rel_pos = Pose()
		rel_pos.x = rel_x
		rel_pos.y = rel_y
		return rel_pos

	def find_target_angle(self, x_3, y_3, target):
		if(target.x - self.pose.x == 0 and target.y - self.pose.y > 0):
					target_angle = np.pi/2
		elif(target.x - self.pose.x == 0 and target.y - self.pose.y < 0):
					target_angle = -1 * np.pi/2
		else:
			x_3 = self.pose.x - target.x
			y_3 = self.pose.y - target.y
			if(y_3 == 0):
				if(x_3 < 0):
					target_angle = 0
				else:
					target_angle = np.pi
			else:
				c_1 = pow(self.pose.x, 2) + pow(self.pose.y, 2) - pow(target.x, 2) - pow(target.y, 2)
				a = (pow(x_3, 2) + pow(y_3, 2))/pow(y_3, 2)
				c_2 = 1/(2 * y_3)*c_1 - self.pose.y
				b = -2 * self.pose.x - 2*c_2*x_3/y_3
				c_3 = pow(self.pose.x, 2) + pow(c_2, 2) - pow(self.radius, 2)
				h = (-1*b + pow(pow(b,2) - 4 * a * c_3,0.5))/(2*a)
				k = -1 * x_3/y_3 * h + 1/(2*y_3)*c_1

			#angle is negative reciprocol of slope b/n target point and radius
			slope = (target.y - k)/(target.x - h)
			if(y_3 > 0):
				target_angle = np.arctan(-1/slope)
			else:
				target_angle = np.arctan(-1/slope) + np.pi
		return target_angle


	def go_to_exp_1(self):
		self.go_to(self.seven, self.four, self.exp)

	def go_to_tea_1(self):
		self.go_to(self.two, self.three, self.six)
		self.go_to(self.five, self.six, self.seven)
		self.go_to(self.eight, self.nine, self.ten)
		self.go_to(self.eleven, self.twelve, self.thirteen)
		self.go_to(self.fourteen, self.fifteen, self.sixteen)
		self.go_to(self.seventeen, self.tea_1, self.tea_1)

	def go_back_to_exp(self):
		self.go_to(self.seventeen, self.sixteen, self.fifteen)
		self.go_to(self.fourteen, self.thirteen, self.twelve)
		self.go_to(self.eleven, self.ten, self.nine)
		self.go_to(self.eight, self.seven, self.six)
		self.go_to(self.five, self.four, self.three)
		self.go_to(self.two, self.exp, self.exp)

if __name__ == '__main__':
	#create the navigation object
	basic_nav = TeaFetching()
	rospy.sleep(3.)
	basic_nav.go_to_exp_1()
	rospy.sleep(2.)
	basic_nav.turn_around_rel(np.pi)
	basic_nav.go_to_tea_1()
	rospy.sleep(2.)
	basic_nav.turn_around_rel(np.pi)
	basic_nav.go_back_to_exp()
	rospy.sleep(2.)
	basic_nav.turn_around_rel(np.pi)
	basic_nav.go_to_tea_1()
	rospy.sleep(2.)

