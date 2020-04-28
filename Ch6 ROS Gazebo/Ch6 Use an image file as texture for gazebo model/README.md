## How to use an image file as texture for gazebo model
```console
1. Create materials/scripts and materials/textures folder
[alonzo@study ~/simulation_ws/src/my_simulations/models/my1stmodel/]$ mkdir scripts texture

2. touch .material file in scripts folder
[alonzo@study ~/simulation_ws/src/my_simulations/models/my1stmodel/scripts]$ touch robot_car.material

3. Edit .material file
[alonzo@study ~/simulation_ws/src/my_simulations/models/my1stmodel/scripts]$ vim robot_car.material

4. Download .png file from my github repo and store it inside the texture folder

5. Modify .sdf file <visual>
[alonzo@study ~/simulation_ws/src/my_simulations/models/my1stmodel]$ vim model.sdf

6. Launch
[alonzo@study ~/simulation_ws/src/my_simulations/models/my1stmodel/]$ roslaunch my_simulations my_world.launch
```

* robot_car.material
```console
material robot_car/Diffuse  # name of your material
{                           # Same as Gazebo/Grey in .sdf file
  recieve_shadows off
  technique
  {
    pass
    {
      texture_unit
      {
        texture seamless_texture.png   # texture file
      }
    }
  }
}
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
          <mesh>
            <uri>model://my1stmodel/meshes/robot_car.stl</uri>
            <!-- Change size to scale -->
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
      <visual name="visual">      <!-- Decide the visual boundary of the object -->
        <geometry>
          <mesh>
            <uri>model://my1stmodel/meshes/robot_car.stl</uri> 
            <!-- Change size to scale -->
            <!-- For cordless_drill scale:1. For robot_car, scale:0.01-->
            <scale>0.01 0.01 0.01</scale>
          </mesh>
        </geometry>
        <material>
          <script>
            <!-- Change Here -->
            <uri>model://my1stmodel/materials/scripts/robot_car.material</uri>
            <uri>model://my1stmodel/materials/textures/seamless_texture.png</uri>
            <name>robot_car/Diffuse</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```
