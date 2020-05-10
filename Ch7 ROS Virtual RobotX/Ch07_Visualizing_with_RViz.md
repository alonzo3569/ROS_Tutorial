## Visualizing with RViz

1. __Launching Gazebo__

```console
$ roslaunch vrx_gazebo vrx.launch camera_enabled:=true gps_enabled:=true imu_enabled:=true
```

2. __Publishing a TF tree__
    * __Node :__ `robot_state_publisher`
    * `robot_state_publisher` will publish all the fixed/non-fixed joints in your URDF to the TF tree based on messages published to `/JointStates`.
    * `robot_state_publisher` is also in charge of deciding the relationship between map/odom/base_link (Rviz Frame).
  * Method 1:
  If you are using the example wamv_gazebo.urdf, run
  
  ```console
  $ roslaunch wamv_gazebo localization_example.launch
  ```
  * Note : Odom frame

  * Method 2: 
  Using a custom URDF without the standard GPS/IMU configuration, run the `robot_state_publisher``
  ```console
  $ rosrun robot_state_publisher robot_state_publisher
  ```
  
3. Running RViz
```console
$ roslaunch wamv_gazebo rviz_vrx.launch
```
