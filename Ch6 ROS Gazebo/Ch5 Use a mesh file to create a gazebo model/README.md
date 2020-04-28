## How to use a mesh file to create a gazebo model
```console
1. Create meshes folders
[alonzo@study ~/simulation_ws/src/my_simulations/models/my1stmodel/]$ mkdir meshes

2. Download .stl file from my github repo to /meshes folder

3. Modify .sdf file, change box to mesh (Both <collision> and <visual>)
[alonzo@study ~/simulation_ws/src/my_simulations/models/my1stmodel/]$ vim model.sdf 

4. Launch file
[alonzo@study ~/simulation_ws/src/my_simulations/models/my1stmodel/]$ roslaunch my_simulations my_world.launch
```

* model.sdf
```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="my1stmodel">        <!-- model name in .world file-->
    <static>true</static>          <!-- can be overwritten by .world file-->
    <link name="link">
      <collision name="collision"> <!-- Decide the collision boundary of the object -->
        <geometry>
          <!-- Change box to mesh -->
          <mesh>
            <uri>model://my1stmodel/meshes/robot_car.stl</uri>
            <!-- Change size to scale -->
            <!-- For cordless_drill scale:1. For robot_car, scale:0.01-->
            <scale>0.01 0.01 0.01</scale>
          </mesh>
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
      <visual name="visual">       <!-- Decide the visual boundary of the object -->
        <geometry>
          <!-- Change box to mesh -->
          <mesh>
            <uri>model://my1stmodel/meshes/robot_car.stl</uri> 
            <!-- Change size to scale -->
            <!-- For cordless_drill scale:1. For robot_car, scale:0.01-->
            <scale>0.01 0.01 0.01</scale>
          </mesh>
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
