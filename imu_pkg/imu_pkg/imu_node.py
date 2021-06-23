#!/usr/bin/env python

#################################################################################
#   Copyright Lars Ludvigsen. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
imu_node.py

"""

import os
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from BMI160_i2c import Driver
from BMI160_i2c import definitions

from deepracer_interfaces_pkg.msg import (IMUSensorMsg)
from imu_pkg import (constants)

class IMUNode(Node):
    """Node responsible for collecting the camera and LiDAR messages and publishing them
       at the rate of the camera sensor.
    """

    def __init__(self):
        """Create a IMUNode.
        """
        super().__init__("imu_node")
        self.get_logger().info("imu_node started.")


        # Publisher that sends combined sensor messages with camera data and LiDAR data
        # to inference node.
        self.imu_message_pub_cb_grp = ReentrantCallbackGroup()
        self.imu_message_publisher = self.create_publisher(IMUSensorMsg,
                                                            constants.IMU_MSG_TOPIC,
                                                            1,
                                                            callback_group=self.imu_message_pub_cb_grp)

    def timer_callback(self):
        """Heartbeat function to keep the node alive.
        """
        self.get_logger().debug(f"Timer heartbeat {self.timer_count}")
        self.timer_count += 1

    def publish_imu_message(self):
        """Publish the sensor message when we get new data for the slowest sensor(LiDAR).
        """
        try:
            sensor_msg = IMUSensorMsg()

            self.imu_message_publisher.publish(sensor_msg)
        except Exception as ex:
            self.get_logger().error(f"Error in publishing sensor message: {ex}")


def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(imu_node, executor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
