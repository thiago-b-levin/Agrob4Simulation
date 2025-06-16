# Webots Sensor Publisher
This package contains the plugins that handles procesing and publishing data from sensors.

## Odometry Publisher:
This Plugin reads data from wheel encoder calculate odometry and publishs the message to a the topic `/odom`.

1. [`sensor_pub.hpp`](./include/wb_sensor_publisher/sensor_pub.hpp): defines plugin namespace, class, parmaeters and functions.
2. [`sensor_pub.hpp`](./include/wb_sensor_publisher/sensor_pub.cpp): plugin implementation.

### URDF Declaration
```xml
  <webots>
    <plugin type="webots_sensor_publisher::sensor_publisher"/>
  </webots>
```

## Point Cloud Publisher

This Plugin reads data from the LiDAR and process it to a `sensor_msgs/PointCloud2` message and this message is published to the topic `/scan`.

1. [`scan_pub.hpp`](./include/wb_sensor_publisher/scan_pub.hpp): defines plugin namespace, class, parmaeters and functions.
2. [`scan_pub.cpp`](./include/wb_sensor_publisher/scan_pub.cpp): plugin implementation.

### URDF Declaration
```xml
  <webots>
    <plugin type="webots_scan_publisher::scan_publisher"/>
  </webots>
```

