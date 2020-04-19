# Chapter **3.**  Customize Your Application With Msg and Srv Files

<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch3%20ROS%20Custom%20Message/ROS_Message_Concept2.PNG"/><br></br>
<img src="https://github.com/alonzo3569/ROS/blob/master/Ch3%20ROS%20Custom%20Message/ROS_Message_Concept.PNG"/><br></br>

</div>

## Create and Build Your Own Custom Msg
```console
# Create a "new" pkg for custom msg and srv
[alonzo@study ~/catkin_ws/src]$ catkin_create_pkg my_robot_msgs roscpp rospy std_msgs
[alonzo@study ~/catkin_ws/src]$ cd /my_robot_msgs

# Remove /include /src directories
[alonzo@study ~/catkin_ws/src/my_robot_msgs]$ rm -rf /include /src

# Create msg/ directory and modify .msg file
[alonzo@study ~/catkin_ws/src/my_robot_msgs]$ mkdir msg;cd msg/
[alonzo@study ~/catkin_ws/src/my_robot_msgs/msg]$ vim HardwareStatus.msg
[alonzo@study ~/catkin_ws/src/my_robot_msgs/msg]$ cd ..

# Modify CMakeLists.txt and package.xml in "Msg pkg"
[alonzo@study ~/catkin_ws/src/my_robot_msgs]$ vim CMakeLists.txt
[alonzo@study ~/catkin_ws/src/my_robot_msgs]$ vim package.xml

# After editing CMakeLists.txt, package.xml and HardwareStatus.msg
# Don't forget to catkin_make & source ~/.bashrc!
[alonzo@study ~/catkin_ws/]$ catkin_make
# Header file will be generated under ~/catkin_ws/devel/include/my_robot_msgs/
[alonzo@study ~/catkin_ws/]$ . ~/bashrc
```
* __CMakeLists.txt__
```
1.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation  <==== Msg class .h/.cpp file will be generated after catkin_make
)

2.
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   HardwareStatus.msg  <====
 )

3.
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

4.
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_robot_msgs
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime   <==== add "message_runtime"
#  DEPENDS system_lib
)
```
* __package.xml__
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

* __HardwareStatus.msg__
```
int64 temperature
bool are_motors_up
string debug_message
```

## Use Your Custom Msg in Your Code
```console
# Include msg pkg in another pkg to use your custom message
[alonzo@study ~/catkin_ws/src/package_name/scripts]$ vim hw_status_publisher.py
[alonzo@study ~/catkin_ws/src/package_name/scripts]$ chmod a+x hw_status_publisher.py
[alonzo@study ~/catkin_ws/src/package_name/scripts]$ vim CMakeList.txt    package.xml
```
* __CMakeLists.txt__
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  rospy_tutorials
  my_robot_msgs    <== add msg pkg
)
```
* __package.xml__
```
<depend>my_robot_msgs</depend>
```
* __hw_status_publisher.py__
```python
#!/usr/bin/env python

import rospy
from my_robot_msgs.msg import HardwareStatus

if __name__ == '__main__':

    rospy.init_node('hardware_status_publisher')

    pub = rospy.Publisher("/my_robot/hardware_status", HardwareStatus, queue_size = 10)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():\
        # Create custom msg object
        msg = HardwareStatus();
        
        # Call data in custom msg object
        msg.temperature = 45
        msg.are_motors_up = True
        msg.debug_message = "Everything is running well"
        pub.publish(msg)
        rate.sleep()
```
## Create Your Own Custom Srv
```console
[alonzo@study ~/catkin_ws/src/my_robot_msgs]$ mkdir srv
[alonzo@study ~/catkin_ws/src/my_robot_msgs/srv]$ vim ComputeDiskArea.srv
[alonzo@study ~/catkin_ws/src/my_robot_msgs/srv]$ vim CMakeList.txt     
[alonzo@study ~/catkin_ws]$ catkin_make
# Check generated header file: cd ~/catkin_ws/devel/include/my_robot_msgs/
[alonzo@study ~/catkin_ws/devel/include/my_robot_msgs]$ ls
# > ComputeDiskAreaRequest.h    ComputeDiskAreaResponse.h
```
* __ComputeDiskArea.srv__
```
float64 radius
---
float64 area
```
* __CMakeLists.txt__
```
## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ComputeDiskArea.srv
 )
 ```
 
## Msg and Srv With Command
```console
[alonzo@study ~]$ rosmsg -h
[alonzo@study ~]$ rosmsg list
[alonzo@study ~]$ rosmsg show 
[alonzo@study ~]$ rossrv -h
 ```
<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch3%20ROS%20Custom%20Message/ROS_Message2.png"/><br></br>
<img src="https://github.com/alonzo3569/ROS/blob/master/Ch3%20ROS%20Custom%20Message/ROS_Message3.png"/><br></br>

</div>

* __Note :__
```python
# spin() only let the node stay active after finishing all the line, doesn't "Loop"
rospy.spin() <== only let the node stay active after finishing all the line, doesn't "Loop"

# 2 ways to implement sleep()
rate = rospy.Rate(10)
rate.sleep()
# Or
rospy.sleep(3)
```

## Practice
* __led_panel.py__
```python
#!/usr/bin/env python

import rospy
from my_robot_msgs.srv import SetLed

led_states = [0,0,0]

def callback_set_led(req):
  	led_number = req.led_number
  	state = req.state
   	global led_states

  	if (led_number > len(led_states)) or (led_number <= 0):
  		return False

  	if not (state == 0 or state == 1):
  		return False

  	led_states[led_number - 1] = state

  	return True

if __name__ == '__main__':
  	rospy.init_node('led_panel')
    
  	server = rospy.Service("/set_led", SetLed, callback_set_led)

	  rate = rospy.Rate(10)

  	while not rospy.is_shutdown():
	  	rospy.loginfo(led_states)
	  	rate.sleep()
```

* __battery.py__
```python
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
```

* __Setled.srv__
```
int64 led_number
int64 state
---
bool success
```
