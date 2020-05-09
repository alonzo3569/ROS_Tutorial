## Thruster Articulation

1. Check out wamv_gazebo_thruster_config.xacro
```console
$ roscd wamv_gazebo/urdf/thruster_layouts
$ vim wamv_gazebo_thruster_config.xacro
# > Copy paste example code below
```
2. Launch
```console
$ roslaunch vrx_gazebo sandisland.launch
```

3. Run rqt_publisher to see params in topic
```console
$ rosrun rqt_publisher rqt_publisher
```

4. Click on the __topic dropdown menu__ and select:
    * Topic `left_thrust_cmd`
    * Topic `left_thrust_angle`
    * Topic `right_thrust_cmd`
    * Topic `right_thrust_angle`
  
5. Press plus sign on the right to display select topic
6. Change the number by double clicking the value of the topic  
7. Click the box start publishing  
* __Note :__ 
  * `Ctrl+Shift+R` to restart wamv position in Gazebo 
  * `left/right_thrust_angle` value range from +1.57 to -1.57 (+90 degrees ~ -90 degrees)
  * If value > +1.57, value = 1.57
  * If value < -1.57, value = -1.57

* wamv_gazebo_thruster_config.xacro 
```xml
<?xml version="1.0"?>
<plugin xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:macro  name="wamv_gazebo_thruster_config" params="name">
    <thruster>
      <!-- Required Parameters -->
      <linkName>${name}_propeller_link</linkName>
      <propJointName>${name}_engine_propeller_joint</propJointName>
      <engineJointName>${name}_chasis_engine_joint</engineJointName>
      <cmdTopic>${name}_thrust_cmd</cmdTopic>
      <!-- Topic that thruster will subscribe to -->
      <!-- Only subscribes if <enableAngle> is true -->
      <angleTopic>${name}_thrust_angle</angleTopic>
      <!-- Enable <angleTopic> -->
      <enableAngle>true</enableAngle>

      <!-- Optional Parameters -->
      <mappingType>1</mappingType>
      <maxCmd>1.0</maxCmd>
      <maxForceFwd>250.0</maxForceFwd>
      <maxForceRev>-100.0</maxForceRev>
      <maxAngle>${pi/2}</maxAngle>
    </thruster>
  </xacro:macro>
</plugin>
```

