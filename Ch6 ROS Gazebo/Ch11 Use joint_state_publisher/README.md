## Use joint_state_publisher

1. Edit rviz.launch file, change gui to "True"  
  * `<param name="use_gui" value="True"/>`

2. Launch rviz
roslaunch my_robot_description rviz.launch
 
3. A joint1 terminal apprears on the screen
4. Drag joint bar to reotate joint1 object

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
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>

  <!-- Show in Rviz   -->
  <!-- Same as rosrun rviz rviz   -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find my_robot_description)/rviz/robot.rviz"/>


</launch>
```
