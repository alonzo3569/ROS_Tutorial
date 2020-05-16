## Use joint_state_publisher

1. Edit rviz.launch file, change gui to "True"  
  * `<param name="use_gui" value="True"/>`

2. Launch rviz
roslaunch my_robot_description rviz.launch
 
3. A joint1 terminal apprears on the screen
4. Drag joint bar to rotate joint1 object

* rviz.launch
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<launch>

  <!-- This will only spawn your robot node, not gazebo -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_robot_description)/urdf/robot.urdf'" />
    
  <!-- send fake joint values -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <!-- Change here -->
    <param name="use_gui" value="True"/>
  </node>

  <!-- Combine joint values -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>

  <!-- Show in Rviz   -->
  <!-- Same as rosrun rviz rviz   -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find my_robot_description)/rviz/robot.rviz"/>


</launch>
```

* __ROS node :__
  * __`joint_state_publisher`:__
    * This package publishes [sensor_msgs/JointState][0] messages for a robot. 
    * The package reads the __robot_description parameter__ from the parameter server, finds all of the __non-fixed joints__ and publishes a JointState message with all those joints defined.
  * __`robot_state_publisher`:__
    * robot_state_publisher calculate the forward kinematics of the robot and publish the results via tf.
    * Source : URDF specified by the parameter robot_description
    * Subscribe topic : `joint_states` (provide joint positions to `robot_state_publisher`)
    * Subscribe message type : sensor_msgs/JointState
  
[0]:http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html
