# Chapter **1.**  ROS Node and Topic

## Install ROS and Setup Your Environment

* [__Installation (ROS Melodic)__][0]

[0]: http://wiki.ros.org/melodic/Installation/Ubuntu

* __Update packages :__
```console
[alonzo@study ~]$ sudo apt-get update
[alonzo@study ~]$ sudo apt-get install ros-melodic-packagename
```

## ROS package
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

## ROS Node

<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch1%20ROS%20Node%20%26%20Topic/ROS_Node_Description.png"/><br></br>
<img src="https://github.com/alonzo3569/ROS/blob/master/Ch1%20ROS%20Node%20%26%20Topic/ROS_Node_Concept.png"/><br></br>

</div> 

* __Python Node:__
  1. Create python node
  ```console
  [alonzo@study ~/catkin_ws/src/package_name]$ mkdir scripts
  [alonzo@study ~/catkin_ws/src/package_name/scripts]$ touch my_first_node.py
  [alonzo@study ~/catkin_ws/src/package_name/scripts]$ chmod a+x my_first_node.py
  [alonzo@study ~/catkin_ws/src/package_name/scripts]$ vim my_first_node.py 
  ```
  2. Edit python node
  ```python
  #! /usr/bin/env python

  import rospy

  if __name__ == '__main__':

      rospy.init_node('my_first_python_node')     # input your node's name

      rospy.loginfo("This node has been started") # log features for ros 

      rate = rospy.Rate(10) # 

      while not rospy.is_shutdown():
          rospy.loginfo("Hello")
          rate.sleep(1)

      rospy.loginfo("Exit now")
  ```
  3. Run python node
  ```console
  [alonzo@study ~/catkin_ws/src/package_name/scripts]$ roscore  # Run roscore anywhere
  [alonzo@study ~/catkin_ws/src/package_name/scripts]$ python my_first_node.py
  Note : rosrun package_name my_first_node.py
  ```

* __C++ Node:__
  1. Create C++ node
  ```cpp
  [alonzo@study ~/catkin_ws/src/package_name/src]$ touch my_first_cpp_node.cpp
  [alonzo@study ~/catkin_ws/src/package_name/src]$ vim CMakeList.txt
  # > Add: 
  # > add_executable({exe_name} src/my_first_cpp_node.cpp)
  # > target_link_libraries({exe_name} ${catkin_LIBRARIES})
  [alonzo@study ~/catkin_ws]$ catkin_make
  ```
  2. Edit C++ node
  ```cpp
  #include <ros/ros.h>

  int main(int argc, char** argv){

      ros::init(argc, argv, "my_first_cpp_node");  // Can't be the same with other nodes
      ros::NodeHandle nh;                          // Start the node
      ROS_INFO("Node has been started");
      //ros::Duration(1.0).sleep();
      ros::Rate rate(10);
      while (ros::ok()){
          ROS_INFO("Hello");
          rate.sleep();
      }
      ROS_INFO("Exit");
  }
  ```
  3. Run C++ node
  ```console
  [alonzo@study ~/catkin_ws]$ catkin_make
  [alonzo@study ~/catkin_ws/devel/lib/package_name]$ ./{exe_name}  # Same as exe_name in pkg CMakeLists
  Note : rosrun package_name {exe_name}
  ```

## ROS Topic

<div align=center>

<img src="https://github.com/alonzo3569/ROS/blob/master/Ch1%20ROS%20Node%20%26%20Topic/ROS_Topic_Description.png"/><br></br>
<img src="https://github.com/alonzo3569/ROS/blob/master/Ch1%20ROS%20Node%20%26%20Topic/ROS_Topic_Concept.png"/><br></br>

</div>

* __Python Publisher/Subscriber:__
  * Publisher node: 
  ```python
  #!/usr/bin/env python

  import rospy
  from std_msgs.msg import String 

  if __name__ == '__main__':

      rospy.init_node('robot_news_radio_transmitter', anonymous = True) # Anonymous: If nodes have the same name, anonymous makes it possible to run both

      pub = rospy.Publisher("/robot_news_radio", String, queue_size=10) #Add buffer for subscriber in case they don't have time to process message

      rate = rospy.Rate(2) #two msg per second

      while not rospy.is_shutdown():
          msg = String() #Create an object from String class, in String class there is a private member called data
          msg.data = "Hi this is Dan fron the Robot News Radio !"
          pub.publish(msg)
          rate.sleep()

      rospy.loginfo("Node wass stopped")
  ```
  * Subscriber node:
  ```python 
  #!/usr/bin/env python

  import rospy
  from std_msgs.msg import String 

  def callback_recieve_radio_data(msg):
    rospy.loginfo("Message recieved : ")
    rospy.loginfo(msg)

  if __name__ == '__main__':

    rospy.init_node('smartphone')

    sub = rospy.Subscriber("/robot_news_radio", String, callback_recieve_radio_data)

    rospy.spin() # Keep running callbcak function
  ```
  

* __C++ Publisher/Subscriber:__
  * Publisher node: 
  ```cpp
  #include <ros/ros.h>
  #include <std_msgs/String.h>
  int main(int argc, char** argv){

    ros::init(argc, argv, "robot_news_radio_transmitter", ros::init_options::AnonymousName); // Can't be the same with other node
    ros::NodeHandle nh; // To start the node

    ros::Publisher pub = nh.advertise<std_msgs::String>("/robot_news_radio", 10);  // (topic name, qeue size)
    ros::Rate rate(3);

    while (ros::ok()){
      std_msgs::String msg;
      msg.data = "Hi this is logan from the robot news radio";
      pub.publish(msg);
      rate.sleep();
      }
  }
  ```
  * Subscriber node:
  ```cpp
  #include <ros/ros.h>
  #include <std_msgs/String.h>

  void callback_recieve_radio_data(const std_msgs::String& msg)
  {
    ROS_INFO("Message recieved : %s", msg.data.c_str());
  }

  int main(int argc, char** argv){

    ros::init(argc, argv, "smartphone"); // Can't be the same with other node
    ros::NodeHandle nh;                  // To start the node

    ros::Subscriber sub = nh.subscribe("/robot_news_radio", 1000, callback_recieve_radio_data); //(topic name, qeue size, function)

    ros::spin();
  }
  ```
