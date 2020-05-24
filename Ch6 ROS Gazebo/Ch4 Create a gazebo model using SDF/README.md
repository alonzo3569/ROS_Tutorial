## Create a gazebo model (Not Robot!) using SDF

* __SDF__ v.s. __URDF__
  * SDF  :  SDF is used for entire robotic simulation environments.
  * URDF :  Only for robot description.

1. **Create a directory for gazebo model in a ros package**
```console
[alonzo@study ~/simulation_ws/src/my_simulations]$ mkdir models
[alonzo@study ~/simulation_ws/src/my_simulations/models]$ mkdir {your_model_name}
[alonzo@study ~/simulation_ws/src/my_simulations/models]$ mkdir my1stmodel
```

2. **Create .sdf and .config**
```console
[alonzo@study ~/simulation_ws/src/my_simulations/models/my1stmodel]$ model.sdf
[alonzo@study ~/simulation_ws/src/my_simulations/models/my1stmodel]$ model.config (paste code)
```

4. **Add model path to env param GAZEBO_MODEL_PATH**
```console
[alonzo@study ~/simulation_ws/src/my_simulations/models/my1stmodel]$ GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{your_model_model_path}
[alonzo@study ~/simulation_ws/src/my_simulations/models/my1stmodel]$ GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ual/simulation_ws/src/my_simulations/models
```
5. **Modify .world file, "include" your model**
```console
[alonzo@study ~/simulation_ws/src/my_simulations/world]$ vim empty_world.world
# > Add : <uri>model://my1stmodel</uri>
```

6. **Launch**
```console
[alonzo@study ~/simulation_ws/src/my_simulations/world]$ roslaunch my_simulations my_world.launch 
```
## Source code
* model.sdf
```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="my1stmodel">   <!-- model name in .world file-->
    <static>true</static>     <!-- can be overwritten by .world file-->
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>3 2 5</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>100</mu>
              <mu2>50</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>3 2 5</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Grey</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

* model.config
```xml
<?xml version="1.0"?>

<model>
  <name>My 1st model</name>
  <version>1.0</version>
  <sdf version="1.5">model.sdf</sdf>

  <author>
    <name>Logan Zhang</name>
    <email>r07525074@ntu.edu.tw</email>
  </author>

  <description>
    This is my first model for gazebo.
  </description>
</model>
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

		<!-- Change here -->
		<!-- Your model -->
		<include>
			<uri>model://my1stmodel</uri> <!-- model name in .sdf file -->
                                                      <!-- Caution! Remember to add model path to env param GAZEBO_MODEL_PATH -->
                        <static>false</static>        <!-- can overwrite static params in .sdf file-->
                        <pose>0 0 5 0 0 0</pose>
		</include>
	</world>
</sdf>
```
