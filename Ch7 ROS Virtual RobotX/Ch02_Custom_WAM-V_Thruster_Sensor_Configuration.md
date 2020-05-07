## Creating a custom WAM-V Thruster and Sensor Configuration For Competition

```console
1. Make a directory for your custom WAM-V 
$ mkdir ~/my_wamv

2. Touch and edit .yaml file
$ vim ~/my_wamv/sensor_config.yaml
$ vim ~/my_wamv/thruster_config.yaml

3. Generate .urdf from launch file
Usage: roslaunch vrx_gazebo generate_wamv.launch thruster_yaml:={ } sensor_yaml:={ } wamv_target:={ }

Params:
    thruster_yaml: Input, the full path of the thruster YAML
    sensor_yaml: Input, the full path of the sensor YAML configuration
    wamv_target: Output, the path where WAM-V URDF will be generated
    
Note: (1) If yaml path is not given, it uses default thruster/sensor yaml
      (2) Create WAM-V with no thruster and no sensors
          thruster_yaml: empty_thruster_config.yaml
          sensor_yaml: empty_sensor_config.yaml

# Case 1
$ roslaunch vrx_gazebo generate_wamv.launch thruster_yaml:=$HOME/my_wamv/thruster_config.yaml sensor_yaml:=$HOME/my_wamv/sensor_config.yaml wamv_target:=$HOME/my_wamv/my_wamv.urdf

# Case2
$ roslaunch vrx_gazebo generate_wamv.launch thruster_yaml:=$HOME/my_wamv/thruster_config.yaml sensor_yaml:=$HOME/my_wamv/sensor_config.yaml wamv_target:=$HOME/my_wamv/my_wamv_2.urdf

# Case 3 (Non-compliant)
$ roslaunch vrx_gazebo generate_wamv.launch thruster_yaml:=$HOME/my_wamv/thruster_config.yaml sensor_yaml:=$HOME/my_wamv/sensor_config.yaml wamv_target:=$HOME/my_wamv/my_wamv_3.urdf


4. Launch the example world with your WAM-V

# Case 1
$ roslaunch vrx_gazebo sandisland.launch urdf:=$HOME/my_wamv/my_wamv.urdf

# Case 2
$ roslaunch vrx_gazebo sandisland.launch urdf:=$HOME/my_wamv/my_wamv_2.urdf

# Case 3
$ roslaunch vrx_gazebo sandisland.launch urdf:=$HOME/my_wamv/my_wamv_3.urdf
```

* sensor_config.yaml
```
wamv_camera:
    - name: front_left_camera
      x: 0.75
      y: 0.1
      P: ${radians(15)}
    - name: front_right_camera
      x: 0.75
      y: -0.1
      P: ${radians(15)}
    - name: middle_right_camera
      x: 0.75
      y: 0.3
      P: ${radians(15)}
wamv_gps:
    - name: gps_wamv
      x: -0.85
wamv_imu:
    - name: imu_wamv
      y: -0.2
wamv_p3d:
    - name: p3d_wamv
lidar:
    - name: lidar_wamv
      type: 16_beam
      P: ${radians(8)}
```
```
# Removed all cameras

wamv_gps:
    - name: gps_wamv
      x: -0.85
wamv_imu:
    - name: imu_wamv
      y: -0.2
wamv_p3d:
    - name: p3d_wamv
lidar:
    - name: lidar_wamv
      type: 16_beam
      P: ${radians(8)}
```
* __Non-compliant__ sensor_config.yaml
```
# Too many cameras
wamv_camera:
    - name: front_left_camera
      x: 0.75
      y: 0.1
      P: ${radians(15)}
    - name: front_right_camera
      x: 0.75
      y: -0.1
      P: ${radians(15)}
    - name: front_far_left_camera
      x: 0.75
      y: 0.3
      P: ${radians(15)}
    - name: front_far_right_camera
      x: 0.75
      y: -0.3
      P: ${radians(15)}
    - name: middle_left_camera
      x: 0.6
      y: 0.4
      P: ${radians(15)}
      Y: ${radians(90)}
      post_Y: ${radians(90)}
```
* thruster_config.yaml
```
engine:
  - prefix: "left"
    position: "-2.373776 1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
  - prefix: "right"
    position: "-2.373776 -1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
```
```
engine:
  - prefix: "left"
    position: "-2.373776 1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
  - prefix: "right"
    position: "-2.373776 -1.027135 0.318237"
    orientation: "0.0 0.0 0.0"

  # Adding new thruster
  - prefix: "middle"
    position: "0 0 0.318237"
    orientation: "0.0 0.0 0.0"
```
* __Non-compliant__ thruster_config.yaml
```
engine:
  - prefix: "left"
    position: "-2.373776 1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
  - prefix: "right"
    position: "-2.373776 -1.027135 0.318237"
    orientation: "0.0 0.0 0.0"

  # Adding new thruster in non-compliant position
  - prefix: "second_right"
    position: "-2.373776 -1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
```

* generate_wamv.launch
```xml
<?xml version="1.0"?>
<launch>
  <arg name="namespace" value="wamv_config"/>

  <group ns="$(arg namespace)">
    <!-- Setup sensor/thruster yaml source path -->
    <!-- Can be modified via command line-->
    <!-- ROS Param: /wamv_config/sensor_yaml: /home/developer/m... -->
    <arg name="sensor_yaml" default="$(find vrx_gazebo)/src/vrx_gazebo_python/generator_scripts/wamv_config/example_sensor_config.yaml"/>
    <param name="sensor_yaml" value="$(arg sensor_yaml)"/>

    <arg name="thruster_yaml" default="$(find vrx_gazebo)/src/vrx_gazebo_python/generator_scripts/wamv_config/example_thruster_config.yaml"/>
    <param name="thruster_yaml" value="$(arg thruster_yaml)"/>

    <!-- ROS Param: /wamv_config/wamv_target: /home/developer/m... -->
    <!-- No default value -->
    <!-- Must input from cmd-line -->
    <arg name="wamv_target"/>
    <param name="wamv_target" value="$(arg wamv_target)"/>

    <param name="wamv_gazebo" value="$(find wamv_gazebo)/urdf/wamv_gazebo.urdf.xacro"/>

    <param name="sensors_dir" value="$(find wamv_gazebo)/urdf/sensors"/>

    <param name="thrusters_dir" value="$(find wamv_description)/urdf/thrusters"/>

    <param name="compliance_dir" value="$(find vrx_gazebo)/src/vrx_gazebo_python/generator_scripts/wamv_config"/>

    <!-- A python execution file "generate_wamv" -->
    <!-- "generate_wamv" import configure_wamv.py file in vrx_gazebo_python.generator_scripts.wamv_config python module-->
    <node name="wamv_generator" pkg="vrx_gazebo" type="generate_wamv" output="screen" required="true"/>
  </group>
</launch>
```

* sandisland.launch
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
  <!-- This will spawn your robot .urdf file from xacro(command="$(find xacro)...), not gazebo -->
  <arg name="urdf" default="$(find wamv_gazebo)/urdf/wamv_gazebo.urdf.xacro"/>
  <param name="robot_description"
         command="$(find xacro)/xacro &#x002D;&#x002D;inorder '$(arg urdf)'
         thruster_config:=$(arg thrust_config)
         camera_enabled:=$(arg camera_enabled)
         gps_enabled:=$(arg gps_enabled)
         imu_enabled:=$(arg imu_enabled)
         lidar_enabled:=$(arg lidar_enabled)
         ground_truth_enabled:=$(arg ground_truth_enabled) "/>

  <!-- Spawn model in Gazebo using "spawn_model" exe file-->
  <!-- Spawn model in Gazebo, script depending on non_competition_mode -->
  <node name="spawn_model" pkg="gazebo_ros" type="spawn_model" if="$(arg non_competition_mode)"
        args="-x $(arg x) -y $(arg y) -z $(arg z)
              -R $(arg R) -P $(arg P) -Y $(arg Y)
              -urdf -param robot_description -model wamv"/>

  <node name="spawn_wamv" pkg="vrx_gazebo" type="spawn_wamv.bash" unless="$(arg non_competition_mode)"
        args="-x $(arg x) -y $(arg y) -z $(arg z)
              -R $(arg R) -P $(arg P) -Y $(arg Y)
              --urdf $(arg urdf) --model wamv"/>
</launch>
```

* __Compliance rules__:
  * All sensors must be contained within one of the sensor bounding boxes.
  * All thrusters must be contained within one of the thruster bounding boxes. 
  * The number of each sensor and thruster in the configuration must be within the limit.
  * For thrusters, there can only be one thruster in each bounding box.
* If you call __generate_wamv.launch__ on __non-compliant configuration YAML files__, red error messages will be printed but the URDF file will still be created. However, it is not a valid configuration for the VRX competition.

* `generate_wamv.launch`: (generate_wamv python exe => import configure_wamv.py)
  * Input: Thruster/sensor configurations YAML files
  * Output: Robot urdf file
  * Check YAML files compliance
  * Generate xacro files from YAML (YAML -> xacro)
  * Generate urdf file (xacro -> urdf -> robot)
  * In fact, it's configure_wamv.py doing all the stuff.(Compliance -> YAML -> xacro -> urdf)
  * To know how to convert YAML to urdf, check out __configure_wamv.py__

* Sensor bounding box:
```
sensor_compliance_for:
        pose: '0.8 0 1.8 0 0 0'
        size: '1 1 1'
        capacity: '-1'
sensor_compliance_aft:
        pose: '-0.9 0 1.8 0 0 0'
        size: '0.5 1 1'
        capacity: '-1'
```
* Thruster bounding box:
```
thruster_compliance_port_aft:
        pose: '-2.25 1 0 0 0 0'
        size: '1 1 1.2'
        capacity: '1'
thruster_compliance_star_aft:
        pose: '-2.25 -1 0 0 0 0'
        size: '1 1 1.2'
        capacity: '1'
thruster_compliance_port_for:
        pose: '1 1 0 0 0 0'
        size: '1 1 1.2'
        capacity: '1'
thruster_compliance_star_for:
        pose: '1 -1 0 0 0 0'
        size: '1 1 1.2'
        capacity: '1'
thruster_compliance_middle:
        pose: 0.25 0 0 0 0 0
        size: '2.5 1 1.2'
        capacity: '1'
```
* Sensor configuration limit
```
wamv_camera: 
    num: 3
    allowed_params:
        x
        y
        z
        R
        P
        Y
        name
        post_Y

wamv_gps:
    num: 1
    allowed_params:
        x
        y
        z
        R
        P
        Y
        name
        post_Y

wamv_imu:
    num: 1
    allowed_params:
        x
        y
        z
        R
        P
        Y
        name
        post_Y

lidar:
    num: 2
    allowed_params:
        x
        y
        z
        R
        P
        Y
        name
        post_Y
        type

wamv_p3d:
    num: 0
    allowed_params:
        name
```
* Thruster configuration limit
```
engine:
    num: 4
    allowed_params:
        prefix
        position
        orientation

```
