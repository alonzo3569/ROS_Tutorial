## Launch Your First Gazebo World Using ROS
```console
1. Create your own workspace and pkg
[alonzo@study ~]$ mkdir -p simulation_ws/src
[alonzo@study ~/simulation_ws/src]$ catkin_create_pkg my_simulations # No dependencies
[alonzo@study ~/simulation_ws]$ catkin_make

2. Create launch and word folder
[alonzo@study ~/simulation_ws/src/my_simulations]$ mkdir launch world

3. Create and edit your .launch file
[alonzo@study ~/simulation_ws/src/my_simulations/launch]$ touch my_world.launch

4. Create and edit your .world file
[alonzo@study ~/simulation_ws/src/my_simulations/world]$ touch empty_world.world

5. Launch your Gazebo world
[alonzo@study ~]$ roslaunch my_simulations my_world.launch
```

* my_world.launch
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
                <arg name="world_name" value="$(arg world)" />
                <arg name="debug" value="$(arg debug)" />
                <arg name="gui" value="$(arg gui)" />
                <arg name="paused" value="$(arg pause)" />
                <arg name="use_sim_time" value="true" />
        </include>
</launch>
```

* empty_world.world
```xml
<?xml version="1.0" ?>

<sdf version="1.5">
	<world name="default">
		<!-- A global light source -->
		<include>
			<uri>model://sun</uri>
		</include>
		<!-- A ground plane -->
		<include>
			<uri>model://ground_plane</uri>
		</include>

	</world>
</sdf>
```
