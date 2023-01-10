# AWS DeepRacer Inertial Measurement Unit

## Overview

The AWS DeepRacer sensor fusion ROS package creates the `imu_node`, which is an additional package for AWS DeepRacer that can be used to provide Acceleration and Gyroscope readings from the built in BMI160 IMU.

This node is responsible for collecting the messages from the IMU and publishing the resulting sensor message. 

## License

The source code is released under [Apache 2.0](https://aws.amazon.com/apache-2-0/).

## Installation
Follow these steps to install the AWS DeepRacer sensor fusion package.

### Prerequisites

The `imu_pkg` specifically depends on the following ROS 2 packages as build and run dependencies:

1. `geometry_msgs`: This package contains the messages for geometric messages.
1. `sensor_msgs`: This package defines messages for commonly used sensors, including cameras and scanning laser rangefinders.

Additionally the following Python Packages are needed:

1. `smbus2` which allows communication via the i2c bus.
1. `BMI160-i2c` which is a driver for the Bosch BMI160 IMU.

## Downloading and building

Open a terminal on the AWS DeepRacer device and run the following commands as the root user.

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Install the Python packages:

        pip install BMI160-i2c smbus2

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the `imu_pkg` on the AWS DeepRacer device:

        git clone https://github.com/larsll/larsll-deepracer-imu-pkg.git

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/larsll-deepracer-imu-pkg
        rosws update

1. Resolve the `imu_pkg` dependencies:

        cd ~/deepracer_ws/larsll-deepracer-imu-pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the `imu_pkg`:

        cd ~/deepracer_ws/larsll-deepracer-imu-pkg && colcon build --packages-select imu_pkg

## Usage

The `imu_node` provides the core functionality to combine the sensor data from various sensors connected to the AWS DeepRacer vehicle. Although the node is built to work with the AWS DeepRacer application, you can run it independently for development, testing, and debugging purposes.

### Run the node

To launch the built `imu_node` as the root user on the AWS DeepRacer device, open another terminal on the AWS DeepRacer device and run the following commands as the root user:

1. Switch to the root user before you source the ROS 2 installation:

        sudo su

1. Source the ROS 2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash 

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/larsll-deepracer-imu-pkg/install/setup.bash

1. Launch the `imu_node` using the launch script:

        ros2 launch imu_pkg imu_pkg_launch.py

## Launch files

The `imu_pkg_launch.py`, included in this package, provides an example demonstrating how to launch the nodes independently from the core application.

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='imu_pkg',
                namespace='imu_pkg',
                executable='imu_node',
                name='imu_node'
            )
        ])

## Node details

### `imu_node`

#### Published topics

| Topic name | Message type | Description |
| ---------- | ------------ | ----------- |
|/`imu_pkg`/`imu_msg`/`raw`|Imu|Publisher that publishes the readings from the IMU in 6 dimensions.|

