## Adding Sensors

Create your own robot description file (URDF) for a custom WAM-V with your choice of sensors. 

1. **Copy `wamv_gazebo wamv_gazebo.urdf.xacro`**
```console
$ roscp wamv_gazebo wamv_gazebo.urdf.xacro my_wamv.urdf.xacro
```

2. **Modify `my_wamv.urdf.xacro` (Remove all sensors on wamv and add lines below)**
```xml
  <xacro:property name="stereo_x" value="1.0" />
  <xacro:wamv_camera name="stereo_left" x="${stereo_x}" y="0.3" z="1.5" P="${radians(15)}" />
  <xacro:wamv_camera name="stereo_right" x="${stereo_x}" y="-0.3" z="1.5" P="${radians(15)}" />
```

* __Note :__
  * A common property "stereo_x" is used so the value is not copied in multiple places
  * The x,y,z and P (pitch) set where the cameras are located **relative to the WAM-V base link**
  * A Python expression ${radians(15)} was used to convert 15 degrees to radians

3. **Compile xacro file**
```console
$ rosrun xacro xacro --inorder my_wamv.urdf.xacro > my_wamv.urdf
```
4. **Launch**
```console
$ roslaunch vrx_gazebo sandisland.launch urdf:=`pwd`/my_wamv.urdf
```
## Source file
* my_wamv.urdf.xacro
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

  <xacro:property name="stereo_x" value="1.0" />
  <xacro:wamv_camera name="stereo_left" x="${stereo_x}" y="0.3" z="1.5" P="${radians(15)}" />
  <xacro:wamv_camera name="stereo_right" x="${stereo_x}" y="-0.3" z="1.5" P="${radians(15)}" />


</robot>
```
