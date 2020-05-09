## Adding Sensors

Create your own robot description file (URDF) for a custom WAM-V with your choice of sensors. 
These sensors will use gazebo to simulate a camera, lidar, GPS, IMU, etc. and publish the corresponding topics to ROS.

```console
$ mkdir example_vrx_package
$ cd example_vrx_package/
$ roscp wamv_gazebo wamv_gazebo.urdf.xacro my_wamv.urdf.xacro
```
```xml
<xacro:include filename="$(find wamv_gazebo)/urdf/wamv_gazebo.xacro"/>
```

The first line includes wamv_gazebo.urdf.xacro. 
This adds the basic WAM-V mesh and joints along with the plugins for dynamics. 
You will likely want to keep this in, unless you are using a different model or dynamics simulation.

```xml
  <xacro:property name="stereo_x" value="1.0" />
  <xacro:wamv_camera name="stereo_left" x="${stereo_x}" y="0.3" z="1.5" P="${radians(15)}" />
  <xacro:wamv_camera name="stereo_right" x="${stereo_x}" y="-0.3" z="1.5" P="${radians(15)}" />
```

A common property "stereo_x" is used so the value is not copied in multiple places
The x,y,z and P (pitch) set where the cameras are located relative to the WAM-V base link
A Python expression ${radians(15)} was used to convert 15 degrees to radians


```console
$ rosrun xacro xacro --inorder my_wamv.urdf.xacro > my_wamv.urdf
$ roslaunch vrx_gazebo sandisland.launch urdf:=`pwd`/my_wamv.urdf
```
__Rviz??__
