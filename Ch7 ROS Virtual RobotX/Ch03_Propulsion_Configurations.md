## Propulsion Configurations

<div align=center>

<img src="https://bitbucket.org/repo/BgXLzgM/images/2101300599-Propulsion%20Options.png"/><br></br>

</div> 

* __"H": Differential Thrust :__ Two, Fixed Stern Thrusters (Default)
```console
$ roslaunch vrx_gazebo sandisland.launch
```

<div align=center>

<img src="https://bitbucket.org/repo/BgXLzgM/images/3341119966-wamv_full_H.png"/><br></br>

</div> 

* __"T": Differential Thrust :__ An Additional Lateral/Bow Thruster
```console
$ roslaunch vrx_gazebo sandisland.launch thrust_config:=T
```

<div align=center>

<img src="https://bitbucket.org/repo/BgXLzgM/images/3753451461-wamv_full_t.png"/><br></br>

</div> 

* __"X": Holonomic Thruster :__ Configuration with Four Fixed Thrusters
```console
$ roslaunch vrx_gazebo sandisland.launch thrust_config:=X
```

<div align=center>

<img src="https://bitbucket.org/repo/BgXLzgM/images/1776480031-wamv_full_x.png"/><br></br>

</div> 
