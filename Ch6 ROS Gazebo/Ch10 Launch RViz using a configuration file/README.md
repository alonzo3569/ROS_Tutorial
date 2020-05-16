## Launch RViz using a configuration file 
* __.rviz configuration file :__ Save rviz config so that you don't have to reconfigure after roslaunch.
```console
1. Launch rviz
[alonzo@study ~/simulation_ws/src/my_robot_description]$ roslaunch my_robot_description rviz.launch

2. Configure your rviz in GUI

3. Select "Save config as" option

4. Store .rivz in ~/simulation_ws/src/my_robot_description/rviz

5. Name it (robot.rviz)

6. Edit rviz.launch file
[alonzo@study ~/simulation_ws/src/my_robot_description/launch]$ vim rviz.launch

7. Relaunch
[alonzo@study ~/simulation_ws/src/my_robot_description/launch]$ roslaunch my_robot_description rviz.launch
```

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
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find my_robot_description)/rviz/robot.rviz"/>


</launch>
```
