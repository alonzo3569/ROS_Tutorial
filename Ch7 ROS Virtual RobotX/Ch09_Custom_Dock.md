## Custom Dock

1. Create a new folder for your custom dock
```console
$ mkdir -p ~/vrx_ws/src/vrx/vrx_gazebo/models/my_dock_base && cd ~/vrx_ws/src/vrx/vrx_gazebo/models/my_dock_base
```

2. Create .config file
```
$ vim model.config
```

3. Edit .config file
```xml
    <?xml version="1.0"?>
    <model>
      <name>my_dock_base</name>
      <version>1.0</version>
      <sdf version="1.6">model.sdf</sdf>

      <author>
        <name>Carlos Agüero</name>
        <email>caguero@openrobotics.org</email>
      </author>

      <description>
        My custom dock base.
      </description>
    </model>
```

4. Create .sdf.erb file
```
$ vim model.sdf.erb
```

5. Edit .sdf.erb file
```xml
    <?xml version="1.0" ?>
    <sdf version="1.6">

      <!-- Important: This file is generated. DO NOT EDIT! -->

      <model name="robotx_dock_2016_base">
    <%
    layout = [
    'X  X',
    'X  X',
    'X  X',
    'XXXXXXX',
    'XXXX',
    'XXXXXXX',
    'X  X',
    'X  X',
    'X  X',
    ]
    %>
    <%= ERB.new(File.read('dock_generator.erb'),
                      nil, '-', 'dock').result(binding) %>
      </model>
    </sdf>
```

6. Open the vrx_gazebo/CMakeLists.txt file and add your model
```
    # Dock base files that need to be processed with erb
    set (dock_base_erb_files
      models/dock_2016_base/model.sdf.erb
      models/dock_2018_base/model.sdf.erb

      # THIS IS THE ONLY LINE THAT NEEDS TO BE ADDED.
      models/my_dock_base/model.sdf.erb   <== Add this line
    )
```

7. Launch gazebo
```
$ roslaunch vrx_gazebo sandisland.launch
```

8. Insert your new dock by clicking on `Insert->my_dock_base`
