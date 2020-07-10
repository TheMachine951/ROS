#!/usr/bin/env python

import rospy
import webbrowser

def start_picam_stream():

	rospy.init_node('picam',anonymous=True)
	rate = rospy.Rate(1)

	webbrowser.open("http://raspi:8000")

	while not rospy.is_shutdown():
		state = "streaming"
		rate.sleep()

if __name__ == '__main__':
	try:
		start_picam_stream()
	except rospy.ROSInterruptException:
		pass
