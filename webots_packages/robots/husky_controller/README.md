# Model E Controller
This package defines the plugin to control the Husky robot and has the launch file to publish the robot state and create a Webots Node to interface with the robot in the simulation.
1. Added to [`husky.urdf`](../husky_description/robot/husky.urdf) the plugin declaration.
```xml
  <webots>
    <plugin type="webots_driver::huskyDriver"/>
  </webots>
``` 
To launch the controller run:
```bash
ros2 launch husky_control run_husky_controll.py
```


