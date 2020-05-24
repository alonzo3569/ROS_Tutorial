## How to visualize a robot URDF using RVIZ
1. **Create rviz.launch in launch folder**
```console
[alonzo@study ~/simulation_ws/src/my_robot_description/launch]$ touch rviz.launch
```

2. **Launch**
```console
[alonzo@study ~/simulation_ws/src/my_robot_description/launch]$ roslaunch my_robot_description rviz.launch
```
3. **In Rviz GUI, press Add**
4. **Choose RobotModel option**
5. **To remove error, change global options "Fixed Frame: map" to link_chassis**

## Source code
* rviz.launch
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<launch>

  <!-- This will only spawn your robot node, not gazebo -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_robot_description)/urdf/robot.urdf'" />
    
  <!-- send fake joint values -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="False"/>
  </node>

  <!-- Combine joint values -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>

  <!-- Show in Rviz   -->
  <!-- Same as rosrun rviz rviz   -->
  <node name="rviz" pkg="rviz" type="rviz" />


</launch>
```
