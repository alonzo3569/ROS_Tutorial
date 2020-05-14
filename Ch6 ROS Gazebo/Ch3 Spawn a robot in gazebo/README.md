## Spawn a robot in gazebo

1. Clone this repository (two-wheeled-robot-simulation pkg)
```console
[alonzo@study ~/simulation_ws/src]$ git clone https://bitbucket.org/theconstructcore/two-wheeled-robot-simulation
```

2. View .xacro file
```console
[alonzo@study ~/simulation_ws/src/two-wheeled-robot-simulation/urdf]$ vim m2wr.xacro
```

3. View .launch file
```console
[alonzo@study ~/simulation_ws/src/two-wheeled-robot-simulation/launch]$ vim spawn.launch
```

4. Launch
```console
[alonzo@study ~/simulation_ws/src/two-wheeled-robot-simulation/launch]$ roslaunch two-wheeled-robot-simulation spawn.launch
```

* m2wr.xacro
```xml
<?xml version="1.0" ?>
<robot name="m2wr" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <xacro:include filename="$(find m2wr_description)/urdf/materials.xacro" />
  <xacro:include filename="$(find m2wr_description)/urdf/m2wr.gazebo" />
  <xacro:include filename="$(find m2wr_description)/urdf/macros.xacro" />
  
  <link name="link_chassis">
    <!-- pose and inertial -->
    <pose>0 0 0.1 0 0 0</pose>
    <inertial>
      <mass value="5"/>
      <origin rpy="0 0 0" xyz="0 0 0.1"/>
      <inertia ixx="0.0395416666667" ixy="0" ixz="0" iyy="0.106208333333" iyz="0" izz="0.106208333333"/>
    </inertial>
    <!-- body -->
    <collision name="collision_chassis">
      <geometry>
        <box size="0.5 0.3 0.07"/>
      </geometry>
    </collision>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.07"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <!-- caster front -->
    <collision name="caster_front_collision">
      <origin rpy=" 0 0 0" xyz="0.35 0 -0.05"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0</mu>
            <mu2>0</mu2>
            <slip1>1.0</slip1>
            <slip2>1.0</slip2>
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name="caster_front_visual">
      <origin rpy=" 0 0 0" xyz="0.2 0 -0.05"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>
  </link>
  
  <link name="sensor_laser">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1" />
      <xacro:cylinder_inertia mass="1" r="0.05" l="0.1" />
    </inertial>
    
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="white" />
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
  </link>
  
  <joint name="joint_sensor_laser" type="fixed">
    <origin xyz="0.15 0 0.05" rpy="0 0 0"/>
    <parent link="link_chassis"/>
    <child link="sensor_laser"/>
  </joint>
  
  <xacro:link_wheel name="link_right_wheel" />
  <xacro:joint_wheel name="joint_right_wheel" child="link_right_wheel" origin_xyz="-0.05 0.20 0" />
  
  <xacro:link_wheel name="link_left_wheel" />
  <xacro:joint_wheel name="joint_left_wheel" child="link_left_wheel" origin_xyz="-0.05 -0.20 0" />
</robot>
```

* spawn.launch
```
<?xml version="1.0" encoding="UTF-8"?>
<launch>

        <!-- overwriting these args -->
        <arg name="debug" default="false" />
        <arg name="gui" default="true" />
        <arg name="pause" default="true" />  <!-- if true, press play button to start simulation -->
        <arg name="world" default="$(find my_simulations)/world/empty_world.world" />

        <!-- include gazebo_ros launcher -->
        <include file="$(find gazebo_ros)/launch/empty_world.launch">
                <arg name="world_name" value="$(arg world)" />
                <arg name="debug" value="$(arg debug)" />
                <arg name="gui" value="$(arg gui)" />
                <arg name="paused" value="$(arg pause)" />
                <arg name="use_sim_time" value="true" />
        </include>


    <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find m2wr_description)/urdf/m2wr.xacro'" />
    
    <arg name="x" default="0"/>
    <arg name="y" default="0"/>
    <arg name="z" default="0.5"/>
    
    <node name="mybot_spawn" pkg="gazebo_ros" type="spawn_model" output="screen"
          args="-urdf -param robot_description -model m2wr -x $(arg x) -y $(arg y) -z $(arg z)" />
    <!-- -model: name of the model -xyz: initial position -->
          
</launch>
```
