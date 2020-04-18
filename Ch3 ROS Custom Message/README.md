# Chapter **3.**  ROS Custom Message

## Customize Your Application With Msg and Srv Files
```console
# Create a pkg for custom messages
[alonzo@study ~/catkin_ws/src]$ catkin_create_pkg my_robot_msgs roscpp rospy std_msgs
[alonzo@study ~/catkin_ws/src]$ cd /my_robot_msgs

# Remove /include /src directories
[alonzo@study ~/catkin_ws/src/my_robot_msgs]$ rm -rf /include /src

# Create msg/ directory and modify .msg file
[alonzo@study ~/catkin_ws/src/my_robot_msgs]$ mkdir msg;cd msg/
[alonzo@study ~/catkin_ws/src/my_robot_msgs/msg]$ vim HardwareStatus.msg
[alonzo@study ~/catkin_ws/src/my_robot_msgs/msg]$ cd ..

# Modify CMakeLists.txt and package.xml
[alonzo@study ~/catkin_ws/src/my_robot_msgs]$ vim CMakeLists.txt
[alonzo@study ~/catkin_ws/src/my_robot_msgs]$ vim package.xml
```

* __CMakeLists.txt__
```
1.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation  <====
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
