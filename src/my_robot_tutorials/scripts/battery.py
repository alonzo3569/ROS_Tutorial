#!/usr/bin/env python

import rospy
from my_robot_msgs.srv import SetLed

def set_led(battery_state):
	rospy.wait_for_service("/set_led")
	try:
		service = rospy.ServiceProxy("/set_led", SetLed)
		state = 0
		if battery_state == 'empty':
			state = 1	
		resp = service(1, state)
		rospy.loginfo("Set led success flag : " + str(resp))
	except rospy.ServiceException as e:
		rospy.logerr(e)


if __name__ == '__main__':
	rospy.init_node('battery')

	battery_state = "full"

	while not rospy.is_shutdown():
		rospy.sleep(7)
		battery_state = "empty"
		rospy.loginfo("The battery is empty !")
		set_led(battery_state)
		rospy.sleep(3)
		battery_state = "full"
		rospy.loginfo("The battery is now full")
		set_led(battery_state)
