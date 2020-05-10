## Plugin Parameters

* __Changing the Wind__
  * __sandisland.world.xacro__ include __usv_wind_plugin.xacro__
  * Determine params such as __wind coefficient__ in __sandisland.world.xacro__
  * Specify that the wind forces should be applied to the `wamv` model(?)

  * Method 1: Modify __usv_wind_plugin.xacro__
  ```xml
  <?xml version="1.0"?>
  <world xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:macro name="usv_wind_gazebo"
                 params="**wind_objs
                           direction:=270
                           mean_vel:=-10  <= Change Here!
                           var_gain:=0    <= Change Here!
                           var_time:=1    <= Change Here!
                           seed:=''
                           ros_update_rate:=10
                           " >
      <!--Gazebo Plugin for simulating WAM-V dynamics-->
      <plugin name="wind" filename="libusv_gazebo_wind_plugin.so">
        <!-- models to be effected by the wind -->
        <xacro:insert_block name="wind_objs"/>
        <!-- Wind -->
        <wind_direction>${direction}</wind_direction> <!-- in degrees -->
        <wind_mean_velocity>${mean_vel}</wind_mean_velocity>
        <var_wind_gain_constants>${var_gain}</var_wind_gain_constants>
        <var_wind_time_constants>${var_time}</var_wind_time_constants>
        <random_seed>${seed}</random_seed> <!-- set to zero/empty to randomize -->
        <update_rate>${ros_update_rate}</update_rate>
      </plugin>
    </xacro:macro>
  </world>
  ```
  Note: Don't forget `catkin_make` before `roslaunch vrx_gazebo sandisland.launch`

  * Method 2: Modify __sandisland.world.xacro__ (X)
  ```xml
  <xacro:include filename="$(find vrx_gazebo)/worlds/xacros/gazebo_wind_plugin.xacro"/>
  <xacro:usv_wind_gazebo>
    <wind_objs>
      <wind_obj>         
        <name>wamv</name>
        <wind_mean_velocity>8</wind_mean_velocity> <= Change Here! (X)
        <link_name>base_link</link_name>
        <coeff_vector>0.5 0.5 0.33</coeff_vector>
      </wind_obj>
    </wind_objs>
  </xacro:usv_wind_gazebo>
  ```
  Note: Any number of wind_obj's may be added under wind_objs. Only one link per model is supported.
  Note: Don't forget `catkin_make` before `roslaunch vrx_gazebo sandisland.launch`

  
  * Ways to change the wind plugin parameters:
    * Change the default wind parameters in __usv_wind_plugin.xacro__
    * Override the default parameters in the __sandisland.world.xacro__
    * Change the __models affected by wind__(?) or their coefficients in __sandisland.world.xacro__
    * Make sure to run `catkin_make` after any changes to process the XML macros.
    
* __Changing the Wave Field__
  * Plugin file: `wave_gazebo/world_models/ocean_waves/model.xacro`
  * World file: `vrx_gazebo/worlds/example_course.world.xacro`
  ```xml
  <!--Waves-->
  <xacro:include filename="$(find wave_gazebo)/world_models/ocean_waves/model.xacro"/>
  <xacro:ocean_waves/>  
  <!-- Call ocean_waves macro using the default input parameters -->
  ```
  
  * Create ane environment without waves:
    * Method 1: Modify `wave_gazebo/world_models/ocean_waves/model.xacro`
    ```xml
     <xacro:macro name="ocean_waves" params="gain:=0.0 period:=5  <= Set gain to zero
                         direction_x:=1.0 direction_y:=0.0
                         angle:=0.4">
    ```
    Note: Don't forget `catkin_make` before `roslaunch vrx_gazebo vrx.launch`

    
    * Method 2: Modify `vrx_gazebo/worlds/example_course.world.xacro`
    ```xml
    <!--Waves-->
      <xacro:include filename="$(find wave_gazebo)/world_models/ocean_waves/model.xacro"/>
      <xacro:ocean_waves gain="0.0"/>  <= Change Here!
    ```
    Note: Don't forget `catkin_make` before `roslaunch vrx_gazebo vrx.launch`
    
* __Changing Fog and Ambient Lighting__
  * Modify `vrx_gazebo/worlds/sandisland.xacro`
  ```xml
      <scene>
      <sky>
        <clouds>
          <speed>12</speed>
        </clouds>
      </sky>
      <grid>0</grid>
      <origin_visual>0</origin_visual>
      <fog>                              => Add <fog> tag
        <type> linear</type>
        <color> 0.7 0.7 0.7 1 </color>
        <density> 0.0 </density>
      </fog>
      <ambient> 1.0 1.0 1.0 1 </ambient> => Add <ambient> tag
    </scene>
  ```
  Note: Don't forget `catkin_make` before `roslaunch vrx_gazebo sandisland.launch`
