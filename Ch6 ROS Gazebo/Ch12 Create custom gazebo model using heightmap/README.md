## Gazebo Heightmap tutorial

1. Install GIMP
https://tecadmin.net/install-gimp-on-ubuntu/

2. Create a New Image

3. Select Width and Height value (2**n+1: 129/129, 257/257, 513/513)
Note : Gazebo requires these values be equal

4. White is high & Black is low. Thus, start with all black.

5. Draw your terrain.

6. Export as .png file

7. In ~/.gazebo/models directory, these are models that you used in the past 
cd ~/.gazebo/models

8. Create a folder for your model
mkdir ~/.gazebo/models/custom_lake

9. Copy model.sdf & model.config from other model
cp -rp ~/.gazebo/models/playground ~/.gazebo/models/custom_lake

10. Overwrite model.sdf file
cp /usr/share/gazebo-9/worlds/heightmap.world model.sdf

11. Edit model.sdf file. Remove <world> tag and "sun" model

12. Remove all the "box" model in model.sdf

13. 

* model.sdf
```xml
<?xml version="1.0" ?>
<sdf version="1.5">
    <!-- Change Here -->
    <model name="custom_lake">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <!-- Change Here -->
              <uri>model://custom_lake/materials/textures/custom_lake.png</uri>
              <!-- Determine the size of terrain-->
              <!-- These value doesn't have to match the size of your image-->
              <!-- 10: The highest point-->
              <size>200 200 10</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual_abcedf">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <texture>
                <diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <texture>
                <diffuse>file://media/materials/textures/grass_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <texture>
                <diffuse>file://media/materials/textures/fungus_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <blend>
                <min_height>2</min_height>
                <fade_dist>5</fade_dist>
              </blend>
              <blend>
                <min_height>4</min_height>
                <fade_dist>5</fade_dist>
              </blend>
              <!-- Change Here -->
              <uri>model://custom_lake/materials/textures/custom_lake.png</uri>
              <!-- Determine the size of terrain-->
              <!-- These value doesn't have to match the size of your image-->
              <!-- 10: The highest point-->
              <size>200 200 10</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
</sdf>
```
