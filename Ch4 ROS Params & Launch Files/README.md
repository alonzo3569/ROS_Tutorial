# Chapter **4.**  ROS Params & Launch Files

<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch4%20ROS%20Params%20%26%20Launch%20Files/ROS_Parameters.PNG"/><br></br>
<img src="https://github.com/alonzo3569/ROS/blob/master/Ch4%20ROS%20Params%20%26%20Launch%20Files/ROS_Parameters_concept.PNG"/><br></br>

</div>


## Manipulate Parameters With Command Line Tools
```console
# Run roscore to initiate up ros parameter server
[alonzo@study ~]$ roscore

# Help for rosparam
# rosparam is a cmd-line tool for setting up ros parameter server
[alonzo@study ~]$ rosparam -h 

# list all the ros params in ros 
[alonzo@study ~]$ rosparam list

# Set ros param
# Usage: rosparam set {param_name} {param_value}
[alonzo@study ~]$ rosparam set /robot_name "my_robot"
[alonzo@study ~]$ rosparam set /sensors_read_frequency 40
[alonzo@study ~]$ rosparam set /simulation_mode false

# Check ros parameters
[alonzo@study ~]$ rosparam list

# Check ros params value
[alonzo@study ~]$ rosparam get /robot_name 
[alonzo@study ~]$ rosparam get /sensors_read_frequency 
[alonzo@study ~]$ rosparam get /simulation_mode
```


## Handle Parameters With Python
```python
# Get param from ros parameter server
publish_frequency = rospy.get_param("/number_publish_frequency")
# Note : remember to set the param in ros server before calling it
# cmd: rosparam set /number_publish_frequency 2

# Set param in py file
rospy.set_param("/another_param", "Hello")
```
```console
# Run .py file and kill it 
# Check in cmd
[alonzo@study ~]$ rosparam list => see "/another_param"
# ros param will till exist even after the node is killed
[alonzo@study ~]$ rosparam get /another_param
```

## Handle Parameters With C++
```cpp
# Get param from ros parameter server
double publish_frequency;
nh.getParam("/number_publish_frequency", publish_frequency);
ros::Rate rate(publish_frequency);
```
```console
# Catkin_make
# Before running node, set ros param first
[alonzo@study ~]$ rosparam set /number_publish_frequency 2

# Check
[alonzo@study ~]$ rosparam list
# Run your node
```
```cpp
# Set param in cpp file
int number;
nh.setParam("/just_another_param","Bye");
```
```console
# Catkin_make
[alonzo@study ~]$ rosparam get /just_another_param

* Note:
Once the node is executed, the value of ros param inside the node is immutable.
Retstart the node to change the value of ros params.
```


## ROS Launch File

<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch4%20ROS%20Params%20%26%20Launch%20Files/ROS_Launch_file_concept.PNG"/><br></br>
<img src="https://github.com/alonzo3569/ROS/blob/master/Ch4%20ROS%20Params%20%26%20Launch%20Files/ROS_Parameters2.PNG"/><br></br>

</div>

## Create a Launch File to Start all Your Parameters and Nodes  
```console
# Create a pkg for launch file
[alonzo@study ~/catkin_ws/src]$ catkin_create_pkg my_robot_bringup
[alonzo@study ~/catkin_ws]$ catkin_make
[alonzo@study ~/catkin_ws/src]$ cd my_robot_bringup
[alonzo@study ~/catkin_ws/src/my_robot_bringup]$ mkdir launch
[alonzo@study ~/catkin_ws/src/my_robot_bringup]$ cd launch
[alonzo@study ~/catkin_ws/src/my_robot_bringup/launch]$ touch number_app.launch
[alonzo@study ~/catkin_ws/src/my_robot_bringup/launch]$ vim number_app.launch
```
* __number_app.launch__
```html
<launch>
    # Add ros params in launch file
    <param name="/number_publish_frequency" type="double" value="3.0" />
    <param name="/number_to_publish" type="int" value="10" />

    # Add node Note "type => exe name"
    <node name="number_publisher" pkg="my_robot_tutorials" type="number_publisher" />
    <node name="number_counter" pkg="my_robot_tutorials" type="number_counter.py" />
</launch>
```
* __Execute launch file__
```console
[alonzo@study ~]$ roslaunch my_robot_bringup number_app.launch

# Kill ros node
# Check ros parameter
[alonzo@study ~]$ rosparams list
[alonzo@study ~]$ rosnode list
[alonzo@study ~]$ rostopic echo  /number
[alonzo@study ~]$ rostopic echo  /number_count

* Note: If using roslaunch, we don't have to run roscore manually
```

