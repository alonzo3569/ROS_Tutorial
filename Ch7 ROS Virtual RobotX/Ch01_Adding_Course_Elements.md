## Adding course elements

1. **Create a ros pkg**
```console
$ catkin_create_pkg example_vrx_package
```

2. **Create a worlds folder for .world and .xacro** 
```console
$ mkdir worlds
```

3. **Copy .xacro file from source pkg**
```console
$ roscp vrx_gazebo example_course.world.xacro .
```

4. **Generate the compiled XML from the xacro file ("compile" your xacro files into static XML)**
```console
$ rosrun xacro xacro --inorder example_course.world.xacro > my_world.world
```

5. **Modify .xcro file (Add new module)**

6. **Run simulation**
```console
$ roslaunch vrx_gazebo sandisland.launch world:=`pwd`/my_world.world
```

## Source code
* __VRX example launch file: (vrx_gazebo) sandisland.launch__
  * Create vrx model from `example_course.world.xacro` (Default)
  * Generate wamv robot urdf using `wamv_gazebo.urdf.xacro` 
  * Spawn wamv robot
  
```xml
<?xml version="1.0"?>
<launch>
  <env name="ROSCONSOLE_CONFIG_FILE" value="$(find vrx_gazebo)/config/custom_rosconsole.conf"/>
  <!-- Gazebo world to load -->
  <arg name="world" default="$(find vrx_gazebo)/worlds/example_course.world" />
  <!-- If true, run gazebo GUI -->
  <arg name="gui" default="true" />
  <!-- If true, run gazebo in verbose mode -->
  <arg name="verbose" default="false"/>
  <!-- If true, start in paused state -->
  <arg name="paused"  default="false"/>
  <!-- Set various other gazebo arguments-->
  <arg name="extra_gazebo_args" default=""/>
  <!-- Start in a default namespace -->
  <arg name="namespace" default="wamv"/>

  <!-- Initial USV location and attitude-->
  <arg name="x" default="158" />
  <arg name="y" default="108" />
  <arg name="z" default="0.1" />
  <arg name="P" default="0" />
  <arg name="R" default="0" />
  <arg name="Y" default="-2.76" />

  <!-- If true, show non-competition ROS topics (/gazebo/model_states, /vrx/debug/wind/direction, etc.)-->
  <arg name="non_competition_mode" default="true"/>
  <arg name="enable_ros_network" value="$(arg non_competition_mode)"/>
  <env name="VRX_DEBUG" value="$(arg non_competition_mode)"/>
  <env unless="$(arg non_competition_mode)" name="GAZEBO_MODEL_PATH" value="$(find vrx_gazebo)/models:$(find wamv_gazebo)/models:$(find wamv_description)/models:$(optenv GAZEBO_MODEL_PATH)"/>

  <!-- Allow user specified thruster configurations
       H = stern trusters on each hull
       T = H with a lateral thruster
       X = "holonomic" configuration -->
  <arg name="thrust_config" default="H" />

  <!-- Do you want to enable sensors? -->
  <arg name="camera_enabled"       default="false" />
  <arg name="gps_enabled"          default="false" />
  <arg name="imu_enabled"          default="false" />
  <arg name="lidar_enabled"        default="false" />
  <arg name="ground_truth_enabled" default="false" />

  <!-- Start Gazebo with the world file -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name"   value="$(arg world)"/>
    <arg name="verbose"      value="$(arg verbose)"/>
    <arg name="paused"       value="$(arg paused)"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui"          value="$(arg gui)" />
    <arg name="enable_ros_network" value="$(arg enable_ros_network)"/>
    <arg name="extra_gazebo_args" value="$(arg extra_gazebo_args)"/>
  </include>

  <!-- Load robot model -->
  <arg name="urdf" default="$(find wamv_gazebo)/urdf/wamv_gazebo.urdf.xacro"/>
  <param name="$(arg namespace)/robot_description"
         command="$(find xacro)/xacro &#x002D;&#x002D;inorder '$(arg urdf)'
         thruster_config:=$(arg thrust_config)
         camera_enabled:=$(arg camera_enabled)
         gps_enabled:=$(arg gps_enabled)
         imu_enabled:=$(arg imu_enabled)
         lidar_enabled:=$(arg lidar_enabled)
         ground_truth_enabled:=$(arg ground_truth_enabled)
         namespace:=$(arg namespace) "/>

  <!-- Spawn model in Gazebo, script depending on non_competition_mode -->
  <node name="spawn_model" pkg="gazebo_ros" type="spawn_model" if="$(arg non_competition_mode)"
        args="-x $(arg x) -y $(arg y) -z $(arg z)
              -R $(arg R) -P $(arg P) -Y $(arg Y)
              -urdf -param $(arg namespace)/robot_description -model wamv"/>

  <node name="spawn_wamv" pkg="vrx_gazebo" type="spawn_wamv.bash" unless="$(arg non_competition_mode)"
        args="-x $(arg x) -y $(arg y) -z $(arg z)
              -R $(arg R) -P $(arg P) -Y $(arg Y)
              --urdf $(arg urdf) --model wamv"/>
</launch>
```

* __VRX example world file : example_course.world.xacro__
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


* __WAMV robot example xacro: wamv_gazebo.urdf.xacro__
```xml
<?xml version="1.0"?>
<!-- Basic WAM-V with gazebo plugins for dynamics -->
<robot xmlns:xacro="http://ros.org/wiki/xacro"
       name="WAM-V">
  <xacro:arg name="locked" default="false" />
  <xacro:arg name="thruster_config" default="H" />
  <xacro:arg name="camera_enabled" default="false" />
  <xacro:arg name="gps_enabled" default="false" />
  <xacro:arg name="imu_enabled" default="false" />
  <xacro:arg name="lidar_enabled" default="false" />
  <xacro:arg name="ground_truth_enabled" default="false" />
  <xacro:arg name="vrx_sensors_enabled" default="false" />
  <xacro:arg name="pinger_enabled" default="false" />
  <xacro:arg name="thruster_namespace" default="thrusters/"/>
  <xacro:arg name="camera_namespace" default="cameras/"/>  
  <xacro:arg name="sensor_namespace" default="sensors/"/>
  <xacro:arg name="pinger_namespace" default="pingers/"/>
  <!-- Note: this is only used for some sensors that do not correctly use the 
  robotNamespace parameter -->
  <xacro:arg name="namespace" default="wamv"/>    
  <xacro:property name="thruster_namespace" value="$(arg thruster_namespace)" scope="global" />
  <xacro:property name="camera_namespace" value="$(arg camera_namespace)" scope="global" />
  <xacro:property name="sensor_namespace" value="$(arg sensor_namespace)" scope="global" />
  <xacro:property name="pinger_namespace" value="$(arg pinger_namespace)" scope="global" />
  <xacro:property name="namespace" value="$(arg namespace)" scope="global" />      
  
  <!-- Sensor yaml file -->
  <xacro:arg name="yaml_sensor_generation" default="false"/>
  <xacro:arg name="sensor_xacro_file" default = ""/>

  <!-- Thruster yaml file -->
  <xacro:arg name="yaml_thruster_generation" default="false"/>
  <xacro:arg name="thruster_xacro_file" default = ""/>

  <!-- === The WAM-V platform === -->
  <xacro:include filename="$(find wamv_gazebo)/urdf/wamv_gazebo.xacro"/>
  
  <!-- === Batteries === -->
  <xacro:include filename="$(find wamv_description)/urdf/battery.xacro"/>
  <xacro:battery prefix="left" position="0 1 0.45" orientation="0 0 0"/>
  <xacro:battery prefix="right" position="0 -1 0.45" orientation="0 0 0"/>

  <!-- === Thrusters === -->
  <!-- Use thruster yaml file if given -->
  <xacro:if value="$(arg yaml_thruster_generation)">
    <xacro:wamv_gazebo thruster_layout="$(arg thruster_xacro_file)"/>
  </xacro:if>
  
  <!-- Otherwise, add thrusters based on thruster_config variable -->
  <xacro:unless value="$(arg yaml_thruster_generation)">
    <xacro:property name="thruster_conf" value="$(arg thruster_config)"/>

    <!-- Default WAM-V with two aft thrusters -->
    <xacro:if value="${thruster_conf == 'H'}">
       <xacro:wamv_gazebo thruster_layout="$(find wamv_gazebo)/urdf/thruster_layouts/wamv_aft_thrusters.xacro"/>
    </xacro:if>

    <!-- WAMV with "T" thruster configuration -->
    <xacro:if value="${thruster_conf == 'T'}">
      <xacro:wamv_gazebo thruster_layout="$(find wamv_gazebo)/urdf/thruster_layouts/wamv_t_thrusters.xacro"/>
    </xacro:if>

    <!-- WAMV with "X" thruster configuration -->
    <xacro:if value="${thruster_conf == 'X'}">
      <xacro:wamv_gazebo thruster_layout="$(find wamv_gazebo)/urdf/thruster_layouts/wamv_x_thrusters.xacro"/>
    </xacro:if>
  </xacro:unless>

  <!-- === Decide if we lock the robot to the world === -->
  <xacro:if value="$(arg locked)">
    <gazebo>
      <link name="wamv_external_link"/>
      <joint name="wamv_external_pivot_joint" type="universal">
        <parent>wamv::base_link</parent>
        <child>wamv_external_link</child>
        <axis>
          <xyz>1 0 0</xyz>
        </axis>
        <axis2>
          <xyz>0 1 0</xyz>
        </axis2>
      </joint>
      <joint name="wamv_external_riser" type="prismatic">
        <parent>world</parent>
        <child>wamv_external_link</child>
        <axis>
          <limit>
            <lower>-3</lower>
            <upper>3</upper>
          </limit>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
    </gazebo>
  </xacro:if>

  <!-- === Sensors === -->
  <!-- Use sensor yaml file if given -->
  <xacro:if value="$(arg yaml_sensor_generation)">
    <xacro:include filename="$(arg sensor_xacro_file)"/>
    <xacro:yaml_sensors />

    <!-- Add CPU Cases -->
    <xacro:include filename="$(find wamv_description)/urdf/cpu_cases.xacro" />
    <xacro:cpu_cases position="-0.15 0 1.53" orientation="0 0 0"/>
  </xacro:if>
  
  <!-- Otherwise, add sensors based on enable variables -->
  <xacro:unless value="$(arg yaml_sensor_generation)">
    <!-- Add a front camera -->
    <xacro:if value="$(arg camera_enabled)">
      <xacro:wamv_camera name="front_camera" y="0.3" x="0.75" P="${radians(15)}" />
    </xacro:if>
    
    <!-- Add simulated GPS -->
    <xacro:if value="$(arg gps_enabled)">
      <xacro:wamv_gps name="gps_wamv" x="-0.85" />
    </xacro:if>
    
    <!-- Add Simulated IMU -->
    <xacro:if value="$(arg imu_enabled)">
      <xacro:wamv_imu name="imu_wamv" y="-0.2" />
    </xacro:if>
    
    <!-- Add 3D LIDAR -->
    <xacro:if value="$(arg lidar_enabled)">
      <xacro:lidar name="lidar_wamv" y="-0.3" type="16_beam"/>
    </xacro:if>
    
    <!-- Add P3D ground truth -->
    <xacro:if value="$(arg ground_truth_enabled)">
      <xacro:wamv_p3d name="p3d_wamv"/>
    </xacro:if>

    <!-- Add pinger -->
    <xacro:if value="$(arg pinger_enabled)">
      <xacro:wamv_pinger name="pinger" position="1.0 0 -1.0" />
    </xacro:if>
    
    <!-- ==== VRX sensor configuration ==== -->
    <xacro:if value="$(arg vrx_sensors_enabled)">
    
      <!-- Add CPU Cases -->
      <xacro:include filename="$(find wamv_description)/urdf/cpu_cases.xacro" />
      <xacro:cpu_cases position="-0.15 0 1.53" orientation="0 0 0"/>
    
      <!-- Add a stereo camera pair -->
      <xacro:wamv_camera name="front_left_camera" y="0.1" x="0.75" P="${radians(15)}" />
      <xacro:wamv_camera name="front_right_camera" y="-0.1" x="0.75" P="${radians(15)}" />
    
      <!-- Add a camera facing right -->
      <xacro:wamv_camera name="middle_right_camera" y="-0.45" P="${radians(15)}" Y="${radians(-90)}" post_Y="${radians(-90)}" />
    
      <!-- Add simulated GPS -->
      <xacro:wamv_gps name="gps_wamv" x="-0.85" />
    
      <!-- Add Simulated IMU -->
      <xacro:wamv_imu name="imu_wamv" y="-0.2" />
    
      <!-- Add 3D LIDAR -->
      <xacro:lidar name="lidar_wamv" type="16_beam"/>

      <!-- Add pinger -->
      <xacro:wamv_pinger name="pinger" position="1.0 0 -1.0" />
      
    </xacro:if>
  </xacro:unless>

</robot>
```

## Working with xacro files
### What is xacro?
* A programmable xml file
### Functionality
* Include other files 
* Define functions, store variables(macros). 
* VRX packages make heavy use of xacro to reduce code reuse and make composing new models/worlds easy.

### Generating XML from xacro
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

### Using Macros  
Here is an example macro included to create a gazebo camera sensor with the ROS plugin.
* __wamv_camera.xacro__ (ROS plugin/Xacro template)
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
* __.urdf file__
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


