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
