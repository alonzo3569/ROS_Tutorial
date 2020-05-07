## Adding course elements
```console
1. Create a ros pkg
catkin_create_pkg example_vrx_package

2. Create a worlds folder for .world and .xacro 
mkdir worlds

3. Copy .xacro file from source pkg
roscp vrx_gazebo example_course.world.xacro .

4. Generate the compiled XML from the xacro file ("compile" your xacro files into static XML)
rosrun xacro xacro --inorder example_course.world.xacro > my_world.world
rosrun xacro xacro --inorder my_wamv.urdf.xacro -o my_wamv.urdf

5. Modify .xcro file (Add new module)

6. Run simulation
roslaunch vrx_gazebo sandisland.launch world:=`pwd`/my_world.world
```


* example_course.world.xacro
```xml
<?xml version="1.0" ?>
<!-- World containing sandisland model and some course challenges -->
<sdf version="1.6" xmlns:xacro="http://ros.org/wiki/xacro">
  <world name="robotx_example_course">

    <!-- Imports the sandisland macro defined in vrx_gazebo -->
    <xacro:include filename="$(find vrx_gazebo)/worlds/sandisland.xacro" />
    <!-- Calls the sandisland macro -->
    <!-- Sets up an empty sand island environment (only water, sky, and coastline) -->
    <xacro:sandisland />
    <!--Waves-->
    <xacro:include filename="$(find wave_gazebo)/world_models/ocean_waves/model.xacro"/>
    <xacro:ocean_waves/>
    <include>
      <uri>model://robotx_navigation_challenge</uri>
      <pose>58 68 2 0 0 0.4</pose>
    </include>
    <include>
      <uri>model://robotx_light_buoy</uri>
      <pose>110 28 0.25 0 0 0</pose>
    </include>
    <include>
      <uri>model://robotx_2016_qualifying_pinger_transit</uri>
      <pose>55 -50 0 0 0 -1.3</pose>
    </include>

    <!-- The 2016 dock with the three placards -->
    <include>
      <uri>model://dock_2016</uri>
      <pose>75 22 0.0 0 0 -2.78</pose>
    </include>

    <!-- The 2018 dock with the two placards -->
    <include>
      <uri>model://dock_2018</uri>
      <pose>-2 -18 0.0 0 0 0.2</pose>
    </include> 
    <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/usv_wind_plugin.xacro"/>
    <xacro:usv_wind_gazebo>
      <wind_objs>
        <wind_obj>
          <name>wamv</name>
          <link_name>base_link</link_name>
          <coeff_vector>.5 .5 .33</coeff_vector>
        </wind_obj>
      </wind_objs>
    </xacro:usv_wind_gazebo>

    <!-- Change Here -->
    <include>
      <!-- Small buoy found in vrx_gazebo -->
      <uri>model://polyform_a3</uri>
      <!-- X Y Z roll pitch yaw, relative to center of gazebo world (a point out in the water) -->
      <pose>-10 2 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://polyform_a7</uri>
      <pose>-15 8 0 0 0 0</pose>
    </include>
    <include>
      <uri>model://surmark950400</uri>
      <pose>-12.5 -3 0 0 0 0</pose>
    </include>

  </world>
</sdf>
</sdf>
```

## Working with xacro files
* __What is xacro?__
  * A programmable xml file
* __Functionality:__
  * Include other files, 
  * Define functions, store variables(macros). 
* The VRX packages make heavy use of xacro to reduce code reuse and make composing new models/worlds easy.

* __Generating XML from xacro__
  * __From Terminal:__ To generate `my_wamv.urdf` from `my_wamv.urdf.xacro`, run this command:  
  `$ rosrun xacro xacro --inorder my_wamv.urdf.xacro -o my_wamv.urdf`
  * __In CMakeLists.txt:__ (Allows you to find this issue at build time, not runtime)
  ```
  find_package(catkin REQUIRED COMPONENTS
    xacro  <= Add this line
  )
  ```
  ```
  xacro_add_files(
    worlds/example_course.world.xacro
    INORDER INSTALL DESTINATION worlds <= This will generate devel/share/worlds/your_project_name/example_course.world in your workspace.
  )
  ```
  * __In launch file:__
  ```
  <?xml version="1.0"?>
  <launch>
    <arg name="urdf" default="$(find my_package)/urdf/wamv_gazebo.urdf"/>
    <param name="robot_description" command="$(find xacro)/xacro --inorder $(arg urdf)"/>
    <!-- ROS PARAMETERS: * /robot_description: <?xml version="1.... -->
  </launch>
  ```

* __Using Macros__  
Here is an example macro included to create a gazebo camera sensor with the ROS plugin.
* wamv_camera.xacro (ROS plugin/Xacro template)
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:macro name="wamv_camera" params="name x:=0.5 y:=0 z:=1.5 R:=0 P:=0 Y:=0">
    <link name="${name}_link"/>
    <joint name="${name}_joint" type="fixed">
      <origin xyz="${x} ${y} ${z}" rpy="${R} ${P} ${Y}"/>
      <parent link="base_link"/>
      <child link="${name}_link"/>
    </joint>
    ...
  </xacro:macro>
</robot>
```
* .urdf file
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro"
       name="WAM-V">
  ...
  <!-- Include the macro -->
  <xacro:include filename="$(find wamv_gazebo)/urdf/sensors/wamv_camera.xacro" />
  <!-- Call the macro to add a front camera -->
  <xacro:wamv_camera name="front_camera"  x="5" Y="1.57" />
  ...
</robot>
```

