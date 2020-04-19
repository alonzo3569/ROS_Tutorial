#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback_receive_radio_data(msg):
	rospy.loginfo("Message received : ")
	rospy.loginfo(msg)


if __name__ == '__main__':
	rospy.init_node('smartphone')
	
	sub = rospy.Subscriber("/robot_news_radio", String, callback_receive_radio_data)

	rospy.spin()
