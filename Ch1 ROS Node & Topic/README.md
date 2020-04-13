# Chapter **1.**  ROS Node and Topic

## Install ROS and Setup Your Environment

* [__Installation (ROS Melodic)__][0]

[0]: http://wiki.ros.org/melodic/Installation/Ubuntu

* __Update packages :__
```console
[alonzo@study ~]$ sudo apt-get update
[alonzo@study ~]$ sudo apt-get update sudo apt-get install ros-melodic-packagename
```

## Run your first ROS program
1. Create a workspace
```console
[alonzo@study ~]$ mkdir ~/catkin_ws
[alonzo@study ~]$ mkdir ~/catkin_ws/src
[alonzo@study ~]$ cd ~/catkin_ws
[alonzo@study ~]$ catkin_make
[alonzo@study ~]$ vim ~/.bashrc
# > Add: source ~/catkin_ws/devel/setup.bash (For exe files based on developer's workspace)
# Note : source /opt/ros/melodic/setup.bash  (For ros global application such as catkin_make, roscore)
[alonzo@study ~]$ source ~/.bashrc
```
2. Create packages
```console
[alonzo@study ~/catkin_ws/src]$ catkin_create_pkg {name of package} {dependencies}
[alonzo@study ~/catkin_ws/src]$ catkin_create_pkg my_robot_tutorials roscpp rospy std_msgs
[alonzo@study ~/catkin_ws/src]$ ls ./my_robot_tutorials
# > CMakeLists.txt include/ package.xml src/ 
[alonzo@study ~/catkin_ws/src]$ cd ~/catkin_ws
[alonzo@study ~/catkin_ws]$ catkin_make
```



