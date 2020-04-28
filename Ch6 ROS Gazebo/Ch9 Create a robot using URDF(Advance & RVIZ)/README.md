## Create a robot using URDF (Advance + RVIZ)
```console
1. Edit .urdf, and link and joint (paste robot.urdf code)
[alonzo@study ~/simulation_ws/src/my_robot_description/urdf]$ vim robot.urdf

2. Launch
[alonzo@study ~/simulation_ws/src/my_robot_description]$ roslaunch my_robot_description rviz.launch

3. In Rviz GUI, press Add
4. Choose RobotModel option
5. To remove error, change global options "Fixed Frame: map" to link_chassis 
```

* robot.urdf
```xml
<?xml version="1.0" ?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Robots are composed by links and joints-->
  <!-- Links are the members of the robot -->
  <!-- Joints connect links together -->
  <!-- First link -->
  <link name="link_chassis">

    <!-- pose and inertial -->
    <pose>0 0 0.1 0 0 0</pose>
    <inertial>
      <mass value="5"/>
      <origin rpy="0 0 0" xyz="0 0 0.1"/>
      <inertia ixx="0.0395416666667" ixy="0" ixz="0" iyy="0.106208333333" iyz="0" izz="0.106208333333"/>
    </inertial>

    <!-- collision -->
    <collision name="collision_chassis">
      <geometry>
        <box size="2 2 1"/>
      </geometry>
    </collision>

    <!-- visual -->
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="2 2 1"/>
      </geometry>
    </visual>

  </link>

  <!-- A joiint between links -->
  <joint name="joint1" type="continuous">
    <origin xyz="0.6 0 0.8" rpy="0 0 0"/>
    <parent link="link_chassis"/>
    <child link="link_arm"/>
  </joint>

  <!-- Second link -->
  <link name="link_arm">

    <!-- pose and inertial -->
    <pose>0 0 0.1 0 0 0</pose>
    <inertial>
      <mass value="5"/>
      <origin rpy="0 0 0" xyz="0 0 0.1"/>
      <inertia ixx="0.0395416666667" ixy="0" ixz="0" iyy="0.106208333333" iyz="0" izz="0.106208333333"/>
    </inertial>

    <!-- collision -->
    <collision name="collision_chassis">
      <geometry>
        <box size="0.2 0.2 1"/>
      </geometry>
    </collision>

    <!-- visual -->
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 1"/>
      </geometry>
    </visual>

  </link>

</robot>
```
