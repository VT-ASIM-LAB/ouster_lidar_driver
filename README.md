Ouster Lidar Driver for CARMA
=============================

This is a fork of the [Ouster ROS Driver for Ouster Sensors](https://github.com/ouster-lidar/ouster-ros) that is used for connecting to and configuring [Ouster](https://ouster.com/) sensors, reading and visualizing data, and interfacing with ROS. This fork has been modified to allow for building a Docker image that can serve as a lidar driver for the [CARMA Platform](https://github.com/usdot-fhwa-stol/carma-platform).

Ubuntu 20.04 Installation
-------------------------
Assuming the CARMA Platform is installed at `~/carma_ws/src`:
```
cd ~/carma_ws/src
git clone https://github.com/VT-ASIM-LAB/ouster_lidar_driver.git
cd ouster_lidar_driver/docker
sudo ./build-image.sh -d
```
After the Docker image is successfully built, add the following lines to the appropriate `docker-compose.yml` file in the `carma-config` directory::
```
ouster-lidar-driver:
  image: usdotfhwastoldev/carma-ouster-lidar-driver:develop
  container_name: ouster-lidar-driver
  network_mode: host
  volumes_from:
    - container:carma-config:ro
  environment:
    - ROS_IP=127.0.0.1
  volumes:
    - /opt/carma/logs:/opt/carma/logs
    - /opt/carma/.ros:/home/carma/.ros
    - /opt/carma/vehicle/calibration:/opt/carma/vehicle/calibration
  command: bash -c '. ./devel/setup.bash && export ROS_NAMESPACE=$${CARMA_INTR_NS} && wait-for-it.sh localhost:11311 -- roslaunch /opt/carma/vehicle/config/drivers.launch drivers:=ouster_lidar'
```
Finally, add the following lines to the `drivers.launch` file in the same directory as `docker-compose.yml`::
```
<include if="$(arg ouster_lidar)" file="$(find ouster_ros)/launch/sensor.launch">
    <arg name="sensor_hostname" value="10.5.5.53"/>
    <arg name="udp_dest" value="10.5.5.1"/>
    <arg name="timestamp_mode" value="TIME_FROM_ROS_TIME"/>
    <arg name="metadata" value="/opt/carma/vehicle/calibration/ouster/OS1-64U.json"/>
    <arg name="viz" value="false"/>
</include>
```
Here it is assumed that `10.5.5.1` is the assigned static IP address of the host computer. `10.5.5.53` should be replaced with the IP address of your lidar sensor and the `metadata` field is where the sensor stores its metadata file. See [this guide](https://github.com/SteveMacenski/ouster_ros1) for more details.

ROS API
-------

### ouster_ros


#### Nodes
* `os_node`
* `os_cloud_node`
* `img_node`

#### Published Topics
Publication frequencies are given for an [Ouster OS1-64U](https://ouster.com/products/scanning-lidar/os1-sensor/) 64-beam uniform lidar sensor.

* `os_node/lidar_packets [ouster_ros/PacketMsg]`: publishes lidar packets received from the sensor (640 Hz).
* `os_node/imu_packets [ouster_ros/PacketMsg]`: publishes IMU packets received from the sensor (100 Hz).
* `os_cloud_node/points [sensor_msgs/PointCloud2]`: publishes the point cloud obtained from one sensor rotation (10 or 20 Hz).
* `os_cloud_node/points2 [sensor_msgs/PointCloud2]`: (if supported by the sensor) publishes the point cloud of second returns obtained from one sensor rotation (10 or 20 Hz).
* `os_cloud_node/imu [sensor_msgs/Imu]`: publishes the IMU data obtained from the sensor (100 Hz).
* `os_cloud_node/discovery [cav_msgs/DriverStatus]`: publishes the CARMA [DriverStatus](https://github.com/usdot-fhwa-stol/carma-msgs/blob/develop/cav_msgs/msg/DriverStatus.msg) message (1.25 Hz).
* `img_node/nearir_image [sensor_msgs/Image]`: publishes the point cloud of received [near infrared photons](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2.1.x.pdf) (related to natural environment illumination) as a 2D (horizontal resolution x number of beams) image (10 or 20 Hz).
* `img_node/range_image [sensor_msgs/Image]`: publishes the point cloud of received [range information](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2.1.x.pdf) as a 2D (horizontal resolution x number of beams) image (10 or 20 Hz).
* `img_node/reflec_image [sensor_msgs/Image]`: publishes the point cloud of calculated [calibrated reflectivity](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2.1.x.pdf) as a 2D (horizontal resolution x number of beams) image (10 or 20 Hz).
* `img_node/signal_image [sensor_msgs/Image]`: publishes the point cloud of received [signal intensity photons](https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2.1.x.pdf) as a 2D (horizontal resolution x number of beams) image (10 or 20 Hz).
* `tf_static [tf2_msgs/TFMessage]`: publishes the relationship of child frames `os_imu` and `os_lidar` with their parent frame `velodyne` (originally `os_sensor`, changed for compatibility with CARMA).

#### Subscribed Topics
* `os_node/lidar_packets [ouster_ros/PacketMsg]`: `os_cloud_node` subscribes to this topic to receive lidar packets.
* `os_node/imu_packets [ouster_ros/PacketMsg]`: `os_cloud_node` subscribes to this topic to receive IMU packets.
* `os_cloud_node/points [sensor_msgs/PointCloud2]`: `img_node` subscribes to this topic to receive the created point cloud.

#### Services
* `os_node/get_metadata [ouster_ros/GetMetadata]`: gets the current sensor metadata.
* `os_node/get_config [ouster_ros/GetConfig]`: gets the current sensor configuration.
* `os_node/set_config [ouster_ros/SetConfig]`: sets a new sensor configuration.

#### Parameters
* `os_node/imu_port`: port to which the sensor should send IMU data.
* `os_node/lidar_mode`: lidar horizontal resolution and rotation rate: either `512x10`, `512x20`, `1024x10`, `1024x20`, `2048x10`, or `4096x5` (if supported by the sensor).
* `os_node/lidar_port`: port to which the sensor should send lidar data.
* `os_node/metadata`: path to read or write metadata file when replaying or receiving sensor data, respectively.
* `os_node/sensor_hostname`: hostname or IP of the sensor in dotted decimal form.
* `os_node/timestamp_mode`: method used to timestamp measurements: TIME_FROM_INTERNAL_OSC, TIME_FROM_SYNC_PULSE_IN, TIME_FROM_PTP_1588, TIME_FROM_ROS_TIME.
* `os_node/udp_dest`: hostname or IP where the sensor will send data packets.
* `os_node/udp_profile_lidar`: lidar packet profile: LEGACY, RNG19_RFL8_SIG16_NIR16_DUAL, RNG19_RFL8_SIG16_NIR16, RNG15_RFL8_NIR8.
* `os_cloud_node/tf_prefix`: namespace for [TF2](http://wiki.ros.org/tf2) transforms.


Examples
--------
See the `sensor.launch` file in the `ouster_ros/launch` directory that is used to launch an [Ouster OS1-64U](https://ouster.com/products/scanning-lidar/os1-sensor/) 64-beam uniform lidar sensor.


Original Ouster Documentation
=============================

[Requirements](#requirements) | [Getting Started](#getting-started) | [Usage](#usage) | [License](#license)


<p style="float: right;"><img width="20%" src="ouster_ros/docs/images/logo.png" /></p>

This ROS package provide support for all Ouster sensors with FW v2.0 or later. Upon launch the driver
will configure and connect to the selected sensor device, once connected the driver will handle
incoming IMU and lidar packets, decode lidar frames and publish corresponding ROS messages on the
topics of `/ouster/imu` and `/ouster/points`. In the case the sensor supports dual return and it was
configured to use this capability, then another topic will published named `/ouster/points2` which
corresponds to the second point cloud.

## Requirements
This driver only supports Melodic and Noetic ROS distros.

In addition to the base ROS installation, the following ROS packages are required:
```bash
sudo apt install -y                     \
    ros-$ROS_DISTRO-pcl-ros             \
    ros-$ROS_DISTRO-rviz                \
    ros-$ROS_DISTRO-tf2-geometry-msgs
```

where `$ROS-DISTRO` is either ``melodic`` or ``noetic``.

Additional dependenices:
```bash
sudo apt install -y \
    build-essential \
    libeigen3-dev   \
    libjsoncpp-dev  \
    libspdlog-dev   \
    cmake
```

## Getting Started
To build the driver using ROS you need to clone the project into the `src` folder of a catkin workspace
as shown below:

```bash
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
```

Next to compile the driver you need to source the ROS environemt into the active termainl:
```bash
source /opt/ros/<ros-distro>/setup.bash # replace ros-distro with 'melodic' or 'noetic'
```

Finally, invoke `catkin_make` command from within the catkin workspace as shown below:
```bash
cd catkin_ws
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Specifying `Release` as the build type is important to have a reasonable performance of the driver.


## Usage
The package supports three modes of interaction, you can connect to a live senosr, replay a recorded bag or record a new
bag file using the corresponding launch files. The commands are listed below

### Sensor Mode
```bash
roslaunch ouster_ros sensor.launch      \
    sensor_hostname:=<sensor host name> \
    metadata:=<json file name>              # metadata is optional
```

### Replay Mode
```bash
roslaunch ouster_ros replay.launch      \
    metadata:=<json file name>          \
    bag_file:=<path to rosbag file>
```

### Recording Mode
```bash
roslaunch ouster_ros record.launch      \
    sensor_hostname:=<sensor host name> \
    metadata:=<json file name>          \
    bag_file:=<optional bag file name>
```

For further detailed instructions refer to the [main guide](./docs/index.rst)


## License
[License File](./LICENSE)
