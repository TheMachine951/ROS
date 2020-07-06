#!/usr/bin/env python2

## Subscribes to driving commands and drives the Pi Bot

# before running this node, ensure you run "$ rosrun joy joy_node" in terminal
# while xbox xontroller is connected

###############################################################################

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
#import RPi.GPIO as GPIO

class Driver:

	def __init__(self):
		
		rospy.init_node('Driver', anonymous=True)
		self.sub = rospy.Subscriber('joy',Joy,self.callback)
		
		rospy.sleep(3)

	def callback(self,message):
		
		self.axes = message.axes
		self.buttons = message.buttons

	def get_commands(self):
		#self.button = self.buttons    #if you want to use the button functions

		self.steer = self.axes[0]      # left stick (left to right, 1 to -1)
		self.throttle = self.axes[5]   # right trigger (up to down, 1 to -1)

		self.steer = self.steer * -1.0
		self.throttle = 127.5 * self.throttle + 127.5

	def drive(self):

		while not rospy.is_shutdown():
			
			self.get_commands()
			print(self.steer); print(self.throttle)
			print(type(self.steer)); print(type(self.throttle))


if __name__ == '__main__':

	user_input = " "
	while user_input != "x":
		user_input = raw_input("Enter 'x' to begin: ")

	print("Wiggle Left Stick during and after countdown.")
	print("5"); rospy.sleep(1)
	print("4"); rospy.sleep(1)
	print("3"); rospy.sleep(1)
	print("2"); rospy.sleep(1)
	print("1"); rospy.sleep(1)
	print("Start Driving the Pi_Bot!")

	driver = Driver()
	driver.drive()