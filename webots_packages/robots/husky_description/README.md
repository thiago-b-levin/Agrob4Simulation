# MODEL-E Description
This package allows to spawn Model E inside the simulation. The robot is defined in the urdf file [`model_e.urdf`](./robot/model_e.urdf).
- [Spawner](#spawner)
- [Sensors](#sensors)
- [Plugins](#plugins)



## Spawner 
In the [launch file](./launch/spawn_model_e.py), added a function from `webots_ros2_driver` called URDF spawner that allows spawning the robot using its urdf file [`husky.urdf`](./robot/husky.urdf).

Function Implementation:

```py
 from webots_ros2_driver.urdf_spawner import URDFSpawner

    spawn_robot = URDFSpawner(
        name='husky',
        urdf_path=robot_description_path, # Path to the URDF file
        translation='-35.5 9 0.14',
        rotation='0 0 1 0',
    )

```

Run:
```bash
ros2 launch husky_description description_launch.py
```

## Sensors

To add a sensor you create a Link and Joint for the sensor and add the gazebo description.

**LiDAR Example**
 ```XML 
    <!-- Velodyne Puck Lidar -->
    <link name="velodyne_lidar">
        <inertial>
            <mass value="0.83"/>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
        </inertial>
        <visual>
          <origin xyz="0 -0.00165 0" rpy="0 0 0"/>
          <geometry>
            <cylinder radius="0.052" length="0.0727"/>
          </geometry>
        </visual>
        <collision>
          <origin xyz="0 -0.00165 0" rpy="0 0 0"/>
          <geometry>
            <cylinder radius="0.052" length="0.0727"/>
          </geometry>
        </collision>
    </link>

    <!-- Attach Lidar to Base -->
    <joint name="velodyne_joint" type="fixed">
        <parent link="base_link"/>
        <child link="velodyne_lidar"/>
        <origin xyz="0 0 0.65" rpy="0 0 0"/>
    </joint>

    <gazebo reference="velodyne_lidar">
      <sensor type="ray" name="velodyne_lidar">
          <pose>0 0 0 0 0 0</pose>
          <visualize>false</visualize>
          <update_rate>10</update_rate>
          <ray>
              <scan>
                  <horizontal>
                      <samples>1875</samples>
                      <resolution>1</resolution>
                      <min_angle>-3.14159</min_angle>
                      <max_angle>3.14159</max_angle>
                  </horizontal>
                  <vertical>
                      <samples>16</samples>
                      <resolution>1</resolution>
                      <min_angle>-0.2618</min_angle>  <!-- -15 degrees -->
                      <max_angle>0.2618</max_angle>   <!-- 15 degrees -->
                  </vertical>
              </scan>
              <range>
                  <min>0.9</min>
                  <max>130</max>
                  <resolution>0.001</resolution>
              </range>
              <noise>
                  <type>gaussian</type>
                  <mean>0.0</mean>
                  <stddev>0.008</stddev>
              </noise>
          </ray>
          <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_velodyne_laser.so">
              <topicName>/velodyne_puck/point_cloud</topicName>
              <frameName>velodyne_lidar</frameName>
              <min_range>0.9</min_range>
              <max_range>130</max_range>
              <gaussianNoise>0.008</gaussianNoise>
          </plugin>
      </sensor>
  </gazebo>
```  



## Plugins
In the beggining of the urdf the plugins are defined:
 ```XML 
  <webots>
    <plugin type="webots_driver::huskyDriver"/>
    <plugin type="webots_sensor_publisher::sensor_publisher">
      <wheel_base>0.512</wheel_base>
      <wheel_radius>0.165</wheel_radius>
      <left_wheel_sensor>left_wheel_sensor</left_wheel_sensor>
      <right_wheel_sensor>right_wheel_sensor</right_wheel_sensor>
      <odom_frame>odom</odom_frame>
      <base_frame>base_link</base_frame>
    </plugin>
    <plugin type="webots_sensor_publisher::imupublisher"/>
    <plugin type="webots_scan_publisher::scan_publisher">
      <lidarName>velodyne_lidar</lidarName>
      <topicName>/scan</topicName>
      <frameName>velodyne_lidar</frameName>
      <updateRate>10</updateRate>
      <alwaysOn>True</alwaysOn>
    </plugin>
    <plugin type="webots_ros2_driver::Ros2IMU">
      <!-- Optional parameters -->
      <topicName>/imu/data</topicName>
      <frameName>imu_link</frameName>
      <updateRate>50</updateRate>
      <alwaysOn>True</alwaysOn>
      <enabled>True</enabled>

      <!-- Required device names (must match Webots names) -->
      <inertialUnitName>imu inertial</inertialUnitName>
      <gyroName>imu gyro</gyroName>
      <accelerometerName>imu accelerometer</accelerometerName>
    </plugin>
    <plugin type="webots_sensor_publisher::camera_publisher">
      <cameraName>camera</cameraName>
      <topicName>/camera</topicName>
      <frameName>camera_link</frameName>  
      <updateRate>30</updateRate>
      <alwaysOn>True</alwaysOn>
    </plugin>
  </webots>
```  

1. **Husky Driver**: is the controller plugin. ([model_e_controller](../model_e_controller/))
2. **Sensor Publisher**: is the plugin that calculates and publish the wheel odometry.([wb_sensor_publisher](../../../wb_sensor_publisher/))
3.  **Scan Publisher**: is the plugin that process the LiDAR information and publishes the Pointcloud2 message.([wb_sensor_publisher](../../../wb_sensor_publisher/))
2.  **ROS2IMU**: is a plugin already available on webots_ros2, that get IMU information and publish it.
5.  **Camera Publisher**: is the plugin that process the Camera information and publishes the compressed image message.([wb_sensor_publisher](../../../wb_sensor_publisher/))




 
