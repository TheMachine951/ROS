#!/usr/bin/env python

## Sergio Esteban (saesteban@cpp.edu)

## Subscribes to driving commands and drives the Pi Bot

# before running this node, run "$ rosrun joy joy_node" in terminal

###############################################################################

# import pertinent libraries 
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO

# set GPIO pin convention
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor controller pins
left_fwd = 19   # Forward - Left
left_rev = 13	# Reverse - Left
right_fwd = 12  # Forward - Right
right_rev = 18  # Reverse - Right

# GPIO setup
GPIO.setup(left_fwd,GPIO.OUT)
GPIO.setup(left_rev,GPIO.OUT)
GPIO.setup(right_fwd,GPIO.OUT)
GPIO.setup(right_rev,GPIO.OUT)

# PWM initializaion
Left_fwd = GPIO.PWM(left_fwd,1000)
Left_rev = GPIO.PWM(left_rev,1000)
Left_fwd.start(0)
Left_rev.start(0)
Right_fwd = GPIO.PWM(right_fwd,1000)
Right_rev = GPIO.PWM(right_rev,1000)
Right_fwd.start(0)
Right_rev.start(0)

class Driver:

	def __init__(self):
		
		# setup for ros master
		rospy.init_node('Driver', anonymous=True)
		self.sub = rospy.Subscriber('joy',Joy,self.callback)
		self.rate = rospy.Rate(15) # hz
		
		# init bias parameters
		self.bias_left = 0
		self.bias_right = 0

		# motor controller objects
		self.left_FWD = Left_fwd
		self.left_REV = Left_rev
		self.right_FWD = Right_fwd
		self.right_REV = Right_rev

		rospy.sleep(3)

	def callback(self,message):
		
		# get controller inputs from /joy topic
		self.axes = message.axes
		self.buttons = message.buttons

	def get_commands(self):

		# get joy stick parameters
		#X = self.buttons[2]     # "X" button
		#B = self.buttons[1]     # "B" button

		steer = self.axes[0]      # left stick (left to right, 1 to -1)
		throttle = self.axes[5]   # right trigger (up to down, 1 to -1)
		D_Pad = self.axes[6]      # D-Pad (left to right, 1 to -1)

		throttle = int(-50.0 * throttle + 50.0)    # remap right trigger (from up to down, 0 to 100)
		steer = int(steer * -throttle)             # remap left stick (left to right, -self.throttle to +self.throttle)

		# get bias to make pi bot run straight

		if D_Pad == 1:
			self.bias_left = self.bias_left - 1
			self.bias_right = self.bias_right + 1
		elif D_Pad == -1:
			self.bias_left = self.bias_left + 1
			self.bias_right = self.bias_right - 1

		# adjust value for turning
		if steer < 0:
			self.left = throttle + steer + self.bias_left
			self.right = throttle + self.bias_right
		elif steer > 0:
			self.left = throttle + self.bias_left
			self.right = throttle - steer + self.bias_right
		else:
			self.left = throttle + self.bias_left
			self.right = throttle + self.bias_right

		# ensure values stay in between (0-100)
		if self.left < 0:
			self.left = 0
		elif self.right < 0:
			self.right = 0
		elif self.left > 100:
			self.left = 100
		elif self.right > 100:
			self.right = 100

		# minimum threshhold value to make motors run (need this because of bias)
		thresh = 20
		if self.left < thresh and self.right < thresh:
			self.left = 0
			self.right = 0

	def Steer_GasPedal(self):

		self.left_FWD.ChangeDutyCycle(self.left)
		self.right_FWD.ChangeDutyCycle(self.right)

		# hardware interaction
		print("Left: " + str(self.left) + "  Right: " + str(self.right) + \
			"  |  Left Bias: " + str(self.bias_left) + "  Right Bias: " + str(self.bias_right))

	def drive(self):

		# main loop
		while not rospy.is_shutdown():
			
			self.get_commands()
			self.Steer_GasPedal()
			self.rate.sleep()

		# Exiting 
		self.left_FWD.stop()
		self.right_FWD.stop()
		GPIO.cleanup()

		print("\n\n Exited! \n\n")


if __name__ == '__main__':

	print("\nWelcome to Pi Bot Driver.")
	print("\n>> To steer, use the left joy stick (LS).")
	print(">> To apply the gas, press the right trigger (RT).")
	print(">> To apply a bias to the steering, press the D-Pad left or right.")

	print("\nEnsure controller is plugged in!")

	user_input = " "
	while user_input != "x":
		user_input = raw_input("\nEnter 'x' to begin: ")

	print("\nLightly tap on right trigger (RT) right after countdown.\n")
	print("3"); rospy.sleep(1)
	print("2"); rospy.sleep(1)
	print("1"); rospy.sleep(1)
	print("\nTap the right trigger (RT)!\n")

	driver = Driver()
	driver.drive()