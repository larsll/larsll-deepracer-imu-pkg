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

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from BMI160_i2c import Driver
from BMI160_i2c import definitions

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Pose
from imu_pkg import (constants)

from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class IMUNode(Node):
    """Node responsible for collecting the camera and LiDAR messages and publishing them
       at the rate of the camera sensor.
    """

    def __init__(self):
        """Create a IMUNode.
        """
        super().__init__("imu_node")
        self.get_logger().info("IMU node initializing.")
        self.stop_queue = threading.Event()

        self.declare_parameter('bus_id', constants.I2C_BUS_ID,
                               ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('address', constants.BMI160_ADDR,
                               ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('publish_rate', constants.IMU_MSG_RATE,
                               ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('zero_motion_odometer', False,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))

        self._bus_id = self.get_parameter('bus_id').value
        self._address = self.get_parameter('address').value
        self._publish_rate = self.get_parameter('publish_rate').value
        self._zero_motion = self.get_parameter('zero_motion_odometer').value

        self.get_logger().info("Connecting to IMU at bus {} address {}".format(self._bus_id, self._address))

        # Publisher that sends combined sensor messages with IMU acceleration and gyroscope data.
        self.imu_message_pub_cb_grp = ReentrantCallbackGroup()
        self.imu_message_publisher = self.create_publisher(Imu,
                                                           constants.IMU_MSG_TOPIC,
                                                           1,
                                                           callback_group=self.imu_message_pub_cb_grp)

        if self._zero_motion:
            self.odom_message_pub_cb_grp = ReentrantCallbackGroup()
            self.odom_message_publisher = self.create_publisher(Odometry,
                                                                constants.ODOM_MSG_TOPIC,
                                                                1,
                                                                callback_group=self.odom_message_pub_cb_grp)

        # Heartbeat timer.
        self.timer_count = 0
        self.timer = self.create_timer(5.0, self.timer_callback)

        self.get_logger().info("IMU node created.")

    def timer_callback(self):
        """Heartbeat function to keep the node alive.
        """
        self.get_logger().debug(f"Timer heartbeat {self.timer_count}")
        self.timer_count += 1

    def __enter__(self):
        """Called when the node object is created using the 'with' statement.
        Returns:
           IMUNode : self object returned.
        """
        try:

            # self.get_logger().info(f"Trying to initialize the sensor at {constants.BMI160_ADDR}
            # on bus {constants.I2C_BUS_ID}")
            self.sensor = Driver(self._address, self._bus_id)  # Depends on changes to library

            # configure sampling rate and filter
            self.sensor.set_accel_rate(6)   # 100Hz
            self.sensor.setAccelDLPFMode(0)

            # Defining the Range for Accelerometer and Gyroscope
            self.sensor.setFullScaleAccelRange(definitions.ACCEL_RANGE_4G, constants.ACCEL_RANGE_4G_FLOAT)
            self.sensor.setFullScaleGyroRange(definitions.GYRO_RANGE_250, constants.GYRO_RANGE_250_FLOAT)

            # Calibrating Accelerometer - assuming that it stands on 'flat ground'.
            # Gravity points downwards, hence Z should be calibrated to -1.
            self.sensor.setAccelOffsetEnabled(True)

            self.sensor.autoCalibrateXAccelOffset(0)
            self.sensor.autoCalibrateYAccelOffset(0)
            self.sensor.autoCalibrateZAccelOffset(-1)

            # Enable standing still check
            if self._zero_motion:
                self.sensor.setZeroMotionDetectionDuration(1)
                self.sensor.setZeroMotionDetectionThreshold(0x02)
                self.sensor.setIntZeroMotionEnabled(True)

        except Exception as ex:
            self.get_logger().info(f"Failed to create IMU monitor: {ex}")
            self.observer = None
            raise ex

        self.get_logger().info('Initialization and calibration of IMU sensor done.')

        self.thread = threading.Thread(target=self.processor)
        self.thread.start()

        # Start IMU event monitor.
        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Exiting.')
        self.stop_queue.set()
        self.rate.destroy()
        self.thread.join()

    def processor(self):

        self.get_logger().info(f"Publishing messages at {self._publish_rate} Hz.")

        if self._zero_motion:
            self.get_logger().info(f"Publishing zero-motion odometry.")

        self.rate = self.create_rate(self._publish_rate)

        while not self.stop_queue.is_set() and rclpy.ok():
            try:
                self.publish_imu_message()
                self.rate.sleep()
            except Exception as ex:
                self.get_logger().error(f"Failed to create IMU message: {ex}")

    def publish_imu_message(self):
        """Publish the sensor message when we get new data for the slowest sensor(LiDAR).
        """
        try:
            imu_msg = Imu()
            data = self.sensor.getMotion6()

            if self._zero_motion:
                nomotion = self.sensor.getIntZeroMotionStatus()

                if nomotion:
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = 'odom'
                    odom_msg.child_frame_id = 'base_link'
                    odom_msg.pose.pose = Pose()
                    odom_msg.pose.covariance = constants.EMPTY_ARRAY_36
                    odom_msg.pose.covariance[0] = -1.0
                    odom_msg.twist.twist.linear = Vector3()
                    odom_msg.twist.covariance = constants.EMPTY_ARRAY_36

                    self.odom_message_publisher.publish(odom_msg)

            # fetch all gyro values - return in rad / sec
            gyro = Vector3()
            # swap x and y
            gyro.x = ((data[1] / constants.CONVERSION_MASK_16BIT_FLOAT) *
                      self.sensor.gyro_range * (math.pi / 180))
            # swap x and y
            gyro.y = ((data[0] / constants.CONVERSION_MASK_16BIT_FLOAT) *
                      self.sensor.gyro_range * (math.pi / 180))
            # upside-down
            gyro.z = ((data[2] / constants.CONVERSION_MASK_16BIT_FLOAT) *
                      self.sensor.gyro_range * (math.pi / 180) * -1.0)

            # fetch all accel values - return in m/s²
            accel = Vector3()
            # swap x and y
            accel.x = (data[4] * (constants.GRAVITY_CONSTANT / constants.CONVERSION_MASK_16BIT_FLOAT) *
                       self.sensor.accel_range)
            # swap x and y
            accel.y = (data[3] * (constants.GRAVITY_CONSTANT / constants.CONVERSION_MASK_16BIT_FLOAT) *
                       self.sensor.accel_range)
            # upside-down
            accel.z = (data[5] * (constants.GRAVITY_CONSTANT / constants.CONVERSION_MASK_16BIT_FLOAT) *
                       self.sensor.accel_range * -1.0)

            imu_msg.angular_velocity = gyro
            imu_msg.angular_velocity_covariance = constants.COVAR_ARRAY_9

            imu_msg.linear_acceleration = accel
            imu_msg.linear_acceleration_covariance = constants.COVAR_ARRAY_9

            imu_msg.orientation_covariance = constants.EMPTY_ARRAY_9
            imu_msg.orientation_covariance[0] = -1.0

            # add header
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'base_link'

            self.get_logger().debug('gz: {:+.0f}'.format(gyro.z))

            self.imu_message_publisher.publish(imu_msg)

        except Exception as ex:
            self.get_logger().error(f"Error in publishing sensor message: {ex}")


def main(args=None):

    try:
        rclpy.init(args=args)
        with IMUNode() as imu_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(imu_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        imu_node.destroy_node()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
