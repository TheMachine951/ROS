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
import time

# set GPIO pin convention
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor controller pins
left_fwd = 19   # Forward - Left
right_fwd = 12  # Forward - Right

# LED Pins
led_green = 21
led_red = 20

# Ultrasonic pins
TRIG = 23
ECHO = 24

# GPIO setup
GPIO.setup(left_fwd,GPIO.OUT)
GPIO.setup(right_fwd,GPIO.OUT)
GPIO.setup(led_green,GPIO.OUT)
GPIO.setup(led_red,GPIO.OUT)
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
GPIO.output(TRIG,False)

# PWM initializaion
Left_fwd = GPIO.PWM(left_fwd,1000)
Left_fwd.start(0)
Right_fwd = GPIO.PWM(right_fwd,1000)
Right_fwd.start(0)


class Driver:

	def __init__(self):
		
		# setup for ros master
		rospy.init_node('Driver', anonymous=True)
		self.sub = rospy.Subscriber('joy',Joy,self.callback)
		self.rate = rospy.Rate(20) # hz

		# motor controller objects
		self.left_FWD = Left_fwd
		self.right_FWD = Right_fwd

		rospy.sleep(3)

	def callback(self,message):
		
		# get controller inputs from /joy topic
		self.axes = message.axes
		#self.buttons = message.buttons

	def driving(self):

		steer = self.axes[0]          # left stick (left to right, 1 to -1)
		throttle = self.axes[5]       # right trigger (up to down, 1 to -1)

		throttle = int(-50.0 * throttle + 50.0)    # remap right trigger (from up to down, 0 to 100)
		steer = steer * -throttle                  # remap steering (from left to right, -throttle to +throttle)

		# adjust value for turning
		if steer < 0:
			self.left = throttle + steer
			self.right = throttle
		elif steer > 0:
			self.left = throttle
			self.right = throttle - steer
		else:
			self.left = throttle
			self.right = throttle

		# turn LEDs on if throttle is applied
		if throttle > 0:
			# turn on green LED
			GPIO.output(led_green,GPIO.HIGH)
			GPIO.output(led_red,GPIO.HIGH)
		else:
			# turn on green LED
			GPIO.output(led_green,GPIO.LOW)
			GPIO.output(led_red,GPIO.LOW)

		# minimum threshhold value to make motors run (motors whine at low PWM)
		thresh = 30
		if self.left < thresh and self.right < thresh:
			self.left = 0
			self.right = 0

		# generate PWM signal and send to the motor controller PWM in range (0.0 - 100.0)
		self.left_FWD.ChangeDutyCycle(self.left)
		self.right_FWD.ChangeDutyCycle(self.right)

	def ultrasonic(self):
		# pinging using the trigger
		ping_time = 0.00001 # 10 micro sec
		GPIO.output(TRIG,True)
		time.sleep(ping_time)
		GPIO.output(TRIG,False)

		# time of flight
		while GPIO.input(ECHO) == False:
			pulse_start = time.time()
		while GPIO.input(ECHO) == True:
			pulse_end = time.time()

	 	#calculate distance
		pulse_duration = pulse_end - pulse_start
		speed_of_sound = 346.0 # [m/s]
		distance_m = (speed_of_sound * pulse_duration) / 2.0
		self.distance_in =  distance_m * 39.37 #[inches]

	def drive(self):

		# main loop
		while not rospy.is_shutdown():
			self.driving()
			self.ultrasonic()
			print "Left: %d  | Right: %d  |  Ultrasonic: %.1f in." % (self.left,self.right,self.distance_in)

			self.rate.sleep()

		# Exiting 
		self.left_FWD.stop()
		self.right_FWD.stop()
		GPIO.output(led_green,GPIO.LOW)
		GPIO.cleanup()

		print("\n\n  Finished Driving! \n\n")


if __name__ == '__main__':

	print("\n   Welcome to Pi Bot Driver.")
	print("\n>> To steer, use the left joy stick (LS).")
	print(">> To apply the gas, press the right trigger (RT).")

	print("\n   Ensure controller is plugged in!")
	print("\n   Lightly tap on the right trigger (RT) right after countdown.\n")

	user_input = " "
	while user_input != "x":
		user_input = raw_input("\nEnter 'x' to begin: ")

	print("\n   3"); rospy.sleep(1)
	print("   2"); rospy.sleep(1)
	print("   1"); rospy.sleep(1)
	print("\n   Tap on the trigger!\n")

	driver = Driver()
	driver.drive()
