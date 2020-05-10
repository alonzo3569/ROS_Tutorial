## Driving

1. Launch simulation
```console
$ roslaunch vrx_gazebo sandisland.launch
```

2. __Teleoperation :__
    1. __rostopic pub__
    ```console
    rostopic pub --once /right_thrust_cmd std_msgs/Float32 "data: 1.0"
    ```
      * This command will timeout after a pre-determined amount of time 
      * Default is 1.0 s, but can be changed in the __wamv_gazebo_thruster_config.xacro__
      * `<maxCmd>1.0</maxCmd>` <br></br>
    2. __Keyboard__
    ```console
    roslaunch vrx_gazebo usv_keydrive.launch
    ```
      * Make sure node `/twist2thrust` is publishing cmd to topic `left/right_thrust_cmd`(which gazebo is subscribing) <br></br>
      
      <div align=center>

      <img src="https://bitbucket.org/repo/BgXLzgM/images/1981347365-key_drive.png"/><br></br>

      </div> 
      
      * If gazebo isn't getting any message from node `/twist2thrust` (as shown in figure)
      * Modify `~/vrx_ws/src/vrx/vrx_gazebo/launch/usv_keydrive.launch`
      * Change `/wamv/thrusters/left_thrust_cmd` to `left_thrust_cmd`
      
      ```xml
      <remap from="left_cmd" to="left_thrust_cmd"/>
      <remap from="right_cmd" to="right_thrust_cmd"/> 
      ```
      <br></br>
      
      <div align=center>

      <img src="https://github.com/alonzo3569/ROS/blob/master/Ch7%20ROS%20Virtual%20RobotX/figure/driving_debug_before.png"/><br></br>

      </div> 
      
      * Debug:
        * `rosrun rqt_graph rqt_graph`
        * `rosnode/rostopic info/list` 
        <br></br>
      
      <div align=center>

      <img src="https://github.com/alonzo3569/ROS/blob/master/Ch7%20ROS%20Virtual%20RobotX/figure/keyboard_debug_after.png"/><br></br>

      </div> 
      
    3. __Gamepad__
    ```console
    $ roslaunch vrx_gazebo usv_joydrive.launch
    ```
     
      
