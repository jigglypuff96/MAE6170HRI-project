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
from random import seed
from random import randint

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
		self.distances = []

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
		self.distances.append(1.0/3.0)

		self.three = Pose()
		self.three.x = 4.0 
		self.three.y = self.two.y - 1.0/3.0
		self.distances.append(1.0/3.0)

		self.four = Pose()
		self.four.x = 4.0
		self.four.y = self.three.y - 1.0/3.0
		self.distances.append(1.0/3.0)

		self.five = Pose()
		self.five.x = 4.0
		self.five.y = self.four.y - 1.0/3.0
		self.distances.append(1.0/3.0)

		self.six = Pose()
		self.six.x = 4.0
		self.six.y = self.five.y - 1.0/3.0
		self.distances.append(1.0/3.0)

		self.seven = Pose()
		self.seven.x = 4.0 
		self.seven.y = self.six.y - 1.0/3.0
		self.distances.append(1.0/3.0)

		self.eight = Pose()
		self.eight.x = 4.0
		self.eight.y = self.seven.y - 1.0/3.0
		self.distances.append(1.0/3.0)

		theta = np.arctan(0.75/3.0)
		d = pow(2 * pow(self.radius, 2) - 2 * self.radius * np.cos(2*theta),0.5)
		self.nine = Pose()
		self.nine.x = self.eight.x + d * np.sin(theta)
		self.nine.y = self.eight.y - d * np.cos(theta)
		self.distances.append(theta/self.radius)
		print self.nine

		self.tea_2 = Pose()
		self.tea_2.x = 6.0
		self.tea_2.y = 2.0

		m = (self.nine.y - self.tea_2.y)/(self.nine.x - self.tea_2.x)
		b = self.tea_2.y - m*self.tea_2.x
		x_int = (self.tea_2.x - self.nine.x)/9

		self.ten = Pose()
		self.ten.x = self.nine.x + x_int
		self.ten.y = m * self.ten.x + b
		diag_dist = pow(pow(self.ten.x - self.nine.x,2)+pow(self.ten.y - self.nine.y,2),0.5)
		self.distances.append(diag_dist)

		self.eleven = Pose()
		self.eleven.x = self.ten.x + x_int
		self.eleven.y = m * self.eleven.x + b
		self.distances.append(diag_dist)

		self.twelve = Pose()
		self.twelve.x = self.eleven.x + x_int
		self.twelve.y = m * self.twelve.x + b
		self.distances.append(diag_dist)

		self.thirteen = Pose()
		self.thirteen.x = self.twelve.x + x_int
		self.thirteen.y = m * self.thirteen.x + b
		self.distances.append(diag_dist)
		
		self.fourteen = Pose()
		self.fourteen.x = self.thirteen.x + x_int
		self.fourteen.y = m * self.fourteen.x + b
		self.distances.append(diag_dist)

		self.fifteen = Pose()
		self.fifteen.x = self.fourteen.x + x_int
		self.fifteen.y = m * self.fifteen.x + b
		self.distances.append(diag_dist)

		self.sixteen = Pose()
		self.sixteen.x = self.fifteen.x + x_int
		self.sixteen.y = m * self.sixteen.x + b
		self.distances.append(diag_dist)

		self.seventeen = Pose()
		self.seventeen.x = self.sixteen.x + x_int
		self.seventeen.y = m * self.seventeen.x + b
		self.distances.append(diag_dist)

		self.distances.append(diag_dist)

		#create the array of speeds used for the path
		self.velspeeds = self.createTimingVariants()

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
			
	def go_to(self, target_1, target_2, target_3, place):
		#create an array of target points along route
		#set up waypoints
		targets = [target_1, target_2, target_3]
		#create a loop that continues a velocity until we reach a target
		x = 0
		while(x < len(targets)):
			dist_to_target = pow((pow(self.pose.x - targets[x].x,2) + pow(self.pose.y - targets[x].y,2)), 0.5)
			#find the target angle with circle geometry equations (center h,k)
			rel_pos = Pose()
			rel_pos.x = targets[x].x - self.pose.x
			rel_pos.y = targets[x].y - self.pose.y
			rel_pos = self.target_rel_robot(rel_pos)
			print rel_pos

			#go straight forward if no angle
			if(abs(rel_pos.y) < 0.35):
				print 'straight'
				while(dist_to_target > 0.015 * (x+1)):
					self.robot_vel_msg.angular.z = 0
					if(place + x < 0):
						self.robot_vel_msg.linear.x = self.vel_const
					else:
						self.robot_vel_msg.linear.x = self.velspeeds[place + x]
					#publish
					self.robot_vel_pub.publish(self.robot_vel_msg)
					self.rate.sleep()
					#update distances
					prev_dist = dist_to_target
					dist_to_target = pow((pow(self.pose.x - targets[x].x,2) + pow(self.pose.y - targets[x].y,2)), 0.5)
					#failsafe, for dead reckoning error
					if(dist_to_target > prev_dist):
						break
					#once a target has been reached, update the velocities to the next target
				x += 1
			#go until angle if you need to turn
			else:
				print 'turning'
				x_3 = targets[x].x - self.pose.x
				y_3 = targets[x].y - self.pose.y
				x_3 = targets[x].x - targets[x-1].x
				y_3 = targets[x].y - targets[x-1].y
				#target_angle = self.find_target_angle(x_3, y_3, targets[x])
				if(x_3 == 0):
					if(y_3 > 0):
						target_angle = -1.0 * np.pi/2.0
					else:
						target_angle = 1.0 * np.pi/2.0
				else:
					target_angle = np.arctan(y_3/x_3)
				rel_angle = self.pose.theta - target_angle

				while(abs(rel_angle) > 0.007):
					if(rel_angle < 0):
						omega = self.velspeeds[place+x]/self.radius
					elif(rel_angle > 0):
						omega = -1 * self.velspeeds[place+x]/self.radius
					self.robot_vel_msg.angular.z = omega
					self.robot_vel_msg.linear.x = self.velspeeds[place+x]		
					#publish
					self.robot_vel_pub.publish(self.robot_vel_msg)
					self.rate.sleep()
					rel_angle = self.pose.theta - target_angle
				x += 1

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
		self.go_to(self.seven, self.four, self.exp, -5)
		self.robot_vel_msg.linear.x = 0
		self.robot_vel_msg.angular.z = 0
		self.robot_vel_pub.publish(self.robot_vel_msg)
		self.rate.sleep()

	def go_to_tea_2(self):
		self.go_to(self.two, self.three, self.four, 0)
		self.go_to(self.five, self.six, self.seven, 3)
		self.go_to(self.eight, self.nine, self.ten, 6)
		self.go_to(self.eleven, self.twelve, self.thirteen, 9)
		self.go_to(self.fourteen, self.fifteen, self.sixteen, 12)
		self.go_to(self.seventeen, self.tea_2, self.tea_2, 15)
		self.robot_vel_msg.linear.x = 0
		self.robot_vel_msg.angular.z = 0
		self.robot_vel_pub.publish(self.robot_vel_msg)
		self.rate.sleep()

	def go_back_to_exp(self):
		self.go_to(self.seventeen, self.sixteen, self.fifteen, 0)
		self.go_to(self.fourteen, self.thirteen, self.twelve, 3)
		self.go_to(self.eleven, self.ten, self.nine, 6)
		self.go_to(self.eight, self.seven, self.six, 9)
		self.go_to(self.five, self.four, self.three, 12)
		self.go_to(self.two, self.exp, self.exp, 15)
		self.robot_vel_msg.linear.x = 0
		self.robot_vel_msg.angular.z = 0
		self.robot_vel_pub.publish(self.robot_vel_msg)
		self.rate.sleep()

	def createTimingVariants(self):
		#create a random array of 17 speeds, 100 times
		timingoptions = self.random_sequence(1000)
		random_trials = 550
		scripted_trials = 0
		total_trials = random_trials + scripted_trials
		for y in range(random_trials):
			a = self.random_sequence(y)
			timingoptions = np.vstack((timingoptions,a))
		
		#create some scripted speed sequences, for validation
		#medium speed
		b = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
		#slow
		c = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
		#fast
		d = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
		#slow-fast
		e = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
		#very jerky
		f = np.array([0.1, 0.9, 0.1, 0.9, 0.1, 0.9, 0.1, 0.9, 0.1, 0.9, 0.1, 0.9, 0.1, 0.9, 0.1, 0.9, 0.1, 0.9])
		#fast-slow
		g = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
		#slow-fast-slow
		h = np.array([0.1, 0.1, 0.1, 0.1, 0.9, 0.9, 0.9, 0.9, 0.9, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
		#fast-slow-fast
		i = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.1, 0.1, 0.1, 0.1, 0.1, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
		#slow to fast continuous
		j = np.array([0.1, 0.1, 0.2, 0.2, 0.3, 0.3, 0.4, 0.4, 0.5, 0.5, 0.6, 0.6, 0.7, 0.7, 0.8, 0.8, 0.9, 0.9])
		#fast to slow continuous
		k = np.array([0.9, 0.9, 0.8, 0.8, 0.7, 0.7, 0.6, 0.6, 0.5, 0.5, 0.4, 0.4, 0.3, 0.3, 0.2, 0.2, 0.1, 0.1])
		
		#add to array
		#timingoptions = np.vstack((timingoptions,b))
		#timingoptions = np.vstack((timingoptions,c))
		#timingoptions = np.vstack((timingoptions,d))
		#timingoptions = np.vstack((timingoptions,e))
		#timingoptions = np.vstack((timingoptions,f))
		#timingoptions = np.vstack((timingoptions,g))
		#timingoptions = np.vstack((timingoptions,h))
		#timingoptions = np.vstack((timingoptions,i))
		#timingoptions = np.vstack((timingoptions,j))
		#timingoptions = np.vstack((timingoptions,k))		

		#find the P(T|q,theta) with the exponential equation and parameters for theta = naturalness
		nat = self.naturalness(total_trials, timingoptions)

		#find the P(T|q,theta) for theta = confidence
		con = self.confidence(total_trials, timingoptions)

		#find the timing with the max & min naturalness
		nat_max = self.find_max(nat)
		nat_min = self.find_min(nat)

		#find the timing with the max & min confidence
		con_max = self.find_max(con)
		con_min = self.find_min(con)

		print 'max con'
		print timingoptions[con_max]
		print 'min con'
		print timingoptions[con_min]

		print 'max nat'
		print timingoptions[nat_max]
		print 'min nat'
		print timingoptions[nat_min]

		#create an aggregate score to minimize conditions that need to be run
		agg = self.aggregate_score(nat, con)
		agg_max = self.find_max(agg)
		agg_min = self.find_min(agg)
		agg_mid = self.find_mid(agg, agg_min, agg_max)
		

		print 'max agg'
		print timingoptions[agg_max]
		print 'min agg'
		print timingoptions[agg_min]
		print 'mid agg'
		print timingoptions[agg_mid]

		#search the arrays for max naturalness, max confidence, min naturalness, min confidence

		#select a row of speeds to implement
		n = con_max
		speeds = np.array([])
		for y in range(18):
			speeds = np.append(speeds, [timingoptions[n,y]])
		return speeds


	def random_sequence(self, base):
		seed(base)
		a = np.array([])
		if(base % 2 == 0):
			for x in range(18):
				n = randint(1,10)/10.0
				if(n > .5):
					n = round(pow(n, 2),1)
				elif(n < 0.5):
					n = round(pow(n, 0.5),1)
				a = np.append(a, [n])
		else:
			for x in range(18):
				n = randint(1,10)/10.0
				a = np.append(a, [n])
		return a

	def naturalness(self, size, data):
		#introduce parameters
		k_high = 100
		k_low = 0.5
		exp = 4.64
		natural_array = []

		#calculate naturalness for each row in the array
		for x in range(size):
			total_j = 0
			for y in range (1,17):
				t_n = self.calc_tot_time(data, x)
				j_i = data[x,y+1] + data[x,y-1] - 2 * data[x,y]
				total_j += pow(j_i,2)
			c_natural_high =  k_high * t_n + total_j
			c_natural_low = k_low * t_n + total_j
			naturalness = self.exponent_criteria(exp, c_natural_low)
			natural_array.append(naturalness)
			#np.sort(natural_array)
		return natural_array

	def confidence(self, size, data):
		#introduce parameters
		r = 100
		#k = 0.6
		k = 0.1
		#exp = 12.9
		exp = 1.9
		#can be 1.0 or 0.5 depending if confidence is high or low
		tau_0 = 0.5
		#frequency 50hz - n_obs is obs / sec
		obs_per_sec = 2
		tau_obs = 0.25
		
		confidence_array = []
		
		#calculate the confidence for each row in the array
		for x in range(size):
			t_n = self.calc_tot_time(data, x)
			n_obs = obs_per_sec * t_n
			tau_f = n_obs * tau_obs + tau_0
			total = 0
			time_index = 0
			for y in range(16):
				time_index += self.distances[y] / data[x,y]
				time_index_2 = time_index + self.distances[y+1] / data[x, y+1]
				total += (time_index_2 - time_index)*(tau_obs/(1+r*abs(data[x,y])))
			tau_f = tau_0 + total
			c = k * t_n + 1 / tau_f
			confidence = self.exponent_criteria(exp, c)
			confidence_array.append(confidence)
		return confidence_array
				
	def exponent_criteria(self, exp, criteria):
		p_hiddenstate = pow(np.e, -1.0 * exp * criteria)
		return p_hiddenstate

	def calc_tot_time(self, timingoptions, row):
		t_n = 0
		for z in range(17):
			t_n += self.distances[z] / timingoptions[row,z]
		return t_n

	def find_max(self, array):
		size = len(array)
		max_val = array[0]
		max_val_loc = 0
		for x in range(size):
			if(array[x] > max_val):
				max_val = array[x]
				max_val_loc = x
		return max_val_loc

	def find_min(self, array):
		size = len(array)
		min_val = array[0]
		min_val_loc = 0
		for x in range(size):
			if(array[x] < min_val):
				min_val = array[x]
				min_val_loc = x
		return min_val_loc

	def find_mid(self, array, min_val, max_val):
		size = len(array)
		middle = int(round(size/2,0))
		#the mid value is the median 
		a = array
		b = array
		b.sort()
		mid_val = b[middle]
		for x in range(size):
			if(a[x] == mid_val):
				median_loc = x
		return x

	def aggregate_score(self, natural_array, confidence_array):
		aggregate_array = []
		nat_weight = pow(10, 15)
		con_weight = 1.0

		for x in range(len(natural_array)):
			#multi-attribute function, weighted sum
			sum_score = pow(natural_array[x], 0.1) + confidence_array[x]
			#multi-attribute function, weighted product
			multi_score = pow(natural_array[x], 0.1) * confidence_array[x]
			aggregate_array.append(multi_score)
		return aggregate_array

if __name__ == '__main__':
	#create the navigation object
	basic_nav = TeaFetching()
		
	rospy.sleep(3.)
	basic_nav.go_to_exp_1()
	rospy.sleep(2.)
	basic_nav.turn_around_rel(np.pi)
	basic_nav.go_to_tea_2()
	rospy.sleep(2.)
	basic_nav.turn_around_rel(np.pi)
	basic_nav.go_back_to_exp()
	rospy.sleep(2.)
	basic_nav.turn_around_rel(np.pi)
	basic_nav.go_to_tea_2()
	rospy.sleep(2.)

