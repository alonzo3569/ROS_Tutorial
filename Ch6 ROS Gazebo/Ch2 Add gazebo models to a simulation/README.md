## Add gazebo models to a simulation
* Online [__Gazebo models__][0]
* Modify your .world file to import gazebo online models
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

                <!-- A SUV -->
		<include>
			<uri>model://suv</uri>      <!-- case-sensitive (should be lowercase)-->
                        <static>false</static>     <!-- decide the object is fixed in one point or not-->
                        <pose>0 0 20 0 0 0</pose>  <!-- x y z row pitch yaw (meters) -->
		</include>
	</world>
</sdf>
```

[0]: https://bitbucket.org/osrf/gazebo_models/src/default/
