## Create a robot using URDF

1. **Create a pkg for your robot**
```console
[alonzo@study ~/simulation_ws/src]$ catkin_create_pkg my_robot_description # No dependcies
```

2. **Create folder urdf & launch in my_robot_description pkg**
```console
[alonzo@study ~/simulation_ws/src/my_robot_description]$ mkdir urdf launch
```
3. **Create .urdf file and edit**
```console
[alonzo@study ~/simulation_ws/src/my_robot_description/urdf]$ touch robot.urdf
```

4. **Create .launch file in launch folder**
```console
[alonzo@study ~/simulation_ws/src/my_robot_description/urdf]$ touch spawn.launch (paste spawn.launch)
```

5. **Launch**
```console
[alonzo@study ~/simulation_ws/src/my_robot_description/urdf]$ roslaunch my_robot_description spawn.launch
```

### Source code
* robot.urdf
```xml
<?xml version="1.0" ?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Robots are composed by links and joints-->
  <!-- Links are the members of the robot -->
  <!-- Joints connect links together -->
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
      <material name="blue"/>
    </visual>

  </link>

</robot>
```

* spawn.launch
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<launch>

       <!-- overwriting these args -->
        <arg name="debug" default="false" />
        <arg name="gui" default="true" />
        <arg name="pause" default="true" />  <!-- if true, press play button to start simulation -->
        <arg name="world" default="$(find my_simulations)/world/empty_world.world" />

        <!-- include gazebo_ros launcher -->
        <include file="$(find gazebo_ros)/launch/empty_world.launch">
                <!-- Remove the line below to load empty_world.launch in gazebo_ros pkg, not my_simulations pkg-->
                <!-- <arg name="world_name" value="$(arg world)" /> -->
                <arg name="debug" value="$(arg debug)" />
                <arg name="gui" value="$(arg gui)" />
                <arg name="paused" value="$(arg pause)" />
                <arg name="use_sim_time" value="true" />
        </include>


    <!-- This will only spawn your robot node, not gazebo -->
    <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_robot_description)/urdf/robot.urdf'" />
    
    <arg name="x" default="0"/>
    <arg name="y" default="0"/>
    <arg name="z" default="0.5"/>
    
    <node name="mybot_spawn" pkg="gazebo_ros" type="spawn_model" output="screen"
          args="-urdf -param robot_description -model my_robot -x $(arg x) -y $(arg y) -z $(arg z)" />
    <!-- -model: name of the model -xyz: initial position -->

</launch>
```
