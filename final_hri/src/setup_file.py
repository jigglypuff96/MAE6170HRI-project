#!/usr/bin/env python

##################################################
# 
# Deanna Kocher & Jijie Zhou
# HRI Final Project - "Experiment Setup"
# Spring 2020
#
##################################################

import rospy
from geometry_msgs.msg import Twist
import geometry_msgs.msg
import numpy as np
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, SetPen
from turtle import Turtle, Screen

class SetupExperiment: 
	def __init__(self):
		#create the setup node
		rospy.init_node('setup', anonymous = True)

		#wait for the spawn service
		rospy.wait_for_service('spawn')
		#create a handle for the spawn service
		self.newturtle = rospy.ServiceProxy('spawn', Spawn)

		#kill the first turtle
		rospy.wait_for_service('kill')
		self.killturtle = rospy.ServiceProxy('kill', Kill)
	
	def setup(self):
		#create the new turtles
		self.newturtle(4,2,np.pi/2, 'robot')
		self.newturtle(8,2,np.pi*3/4, 'participant')
		self.newturtle(4,9,np.pi/2, 'experimenter')
		self.newturtle(8,4,np.pi/4, 'earl_gray')
		self.newturtle(6,2,np.pi*5/4, 'green_tea')

		self.killturtle('turtle1')

		#wait for Set Pen service
		rospy.wait_for_service('/robot/set_pen')
		self.setpen = rospy.ServiceProxy('/robot/set_pen', SetPen)
		self.setpen(0, 0, 0, 1, 1)

if __name__ == '__main__':
	#start the new node, run setup
	setupexperiment = SetupExperiment()
	setupexperiment.setup()
	
