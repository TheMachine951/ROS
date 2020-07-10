#!/usr/bin/env python2

import rospy
import os

def start_picam_stream():

	rospy.init_node('picam',anonymous=True)
	rate = rospy.Rate(1)

	# bash script commands
	bash_cmd = "cd /home/sergio/catkin_ws/src/pi_bot/src && python3 picam_start_stream.py"
	os.system(bash_cmd)

	while not rospy.is_shutdown():
		state = "streaming"
		rate.sleep()

if __name__ == '__main__':
	try:
		start_picam_stream()
	except rospy.ROSInterruptException:
		pass
