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
from std_msgs.msg import Header
from geometry_msgs.msg import (Quaternion, Vector3)
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
        self.stop_queue = threading.Event()
        ## following params can now be configured in config file config/imu_params.yaml
        self.imu_frame = self.declare_parameter("~imu_frame", 'imu_link').value
        self.pub_topic = self.declare_parameter("~imu_pub_topic", '/imu_msg/raw').value
        
        ## hex to decimal of 0x69 = 105, change it in config file as needed
        self.imu_i2c_address = self.declare_parameter("~device_address", 105).value
        self.imu_i2c_bus_id = self.declare_parameter("~bus_id", 1).value
        self.publish_rate = self.declare_parameter("~publish_rate", 25).value
        self.get_logger().info(f"imu_frame:: {self.imu_frame} pub_topic:: {self.pub_topic} publish_rate::{self.publish_rate}")

        # Publisher that sends combined sensor messages with IMU acceleration and gyroscope data.
        self.imu_message_pub_cb_grp = ReentrantCallbackGroup()
        self.imu_message_publisher = self.create_publisher(Imu,
                                                            self.pub_topic,
                                                            1,
                                                            callback_group=self.imu_message_pub_cb_grp)

        # Heartbeat timer.
        self.timer_count = 0
        self.timer = self.create_timer(5.0, self.timer_callback)

        self.get_logger().info(f"IMU node successfully created. publishing on topic:: {self.pub_topic}")


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

            # self.get_logger().info(f"Trying to initialize the sensor at {constants.BMI160_ADDR} on bus {constants.I2C_BUS_ID}")
            self.sensor = Driver(self.imu_i2c_address, self.imu_i2c_bus_id) # Depends on changes to library

            # Defining the Range for Accelerometer and Gyroscope
            self.sensor.setFullScaleAccelRange(definitions.ACCEL_RANGE_4G, constants.ACCEL_RANGE_4G_FLOAT)
            self.sensor.setFullScaleGyroRange(definitions.GYRO_RANGE_250, constants.GYRO_RANGE_250_FLOAT)

            # Calibrating Accelerometer - assuming that it stands on 'flat ground'.
            # Gravity points downwards, hence Z should be calibrated to -1.
            self.sensor.autoCalibrateXAccelOffset(0)
            self.sensor.autoCalibrateYAccelOffset(0)
            self.sensor.autoCalibrateZAccelOffset(-1)

            self.sensor.setAccelOffsetEnabled(True)

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

        self.get_logger().info(f"Publishing messages at {self.publish_rate} Hz.")
        self.rate = self.create_rate(self.publish_rate)

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

            # fetch all gyro values - return in rad / sec
            # update for fork - original repo had x & y swapped and z values reversed/upside down. Due to this the IMU data was not showing correct
            # so fixed the swap and z direction for both gyro and accel values

            gyro = Vector3()
            gyro.x = data[0] / constants.CONVERSION_MASK_16BIT_FLOAT * constants.GYRO_RANGE_250_FLOAT * (math.pi / 180) 
            gyro.y = data[1] / constants.CONVERSION_MASK_16BIT_FLOAT * constants.GYRO_RANGE_250_FLOAT * (math.pi / 180) 
            gyro.z = data[2] / constants.CONVERSION_MASK_16BIT_FLOAT * constants.GYRO_RANGE_250_FLOAT * (math.pi / 180) 
            
            # fetch all accel values - return in m/s²
            accel = Vector3()
            accel.x = data[3] * constants.GRAVITY_CONSTANT / constants.CONVERSION_MASK_16BIT_FLOAT * constants.ACCEL_RANGE_4G_FLOAT 
            accel.y = data[4] * constants.GRAVITY_CONSTANT / constants.CONVERSION_MASK_16BIT_FLOAT * constants.ACCEL_RANGE_4G_FLOAT 
            accel.z = data[5] * constants.GRAVITY_CONSTANT / constants.CONVERSION_MASK_16BIT_FLOAT * constants.ACCEL_RANGE_4G_FLOAT 
            
            imu_msg.angular_velocity = gyro
            imu_msg.angular_velocity_covariance = constants.EMPTY_ARRAY_9
            imu_msg.linear_acceleration = accel
            imu_msg.linear_acceleration_covariance = constants.EMPTY_ARRAY_9
            
            imu_msg.orientation_covariance = constants.EMPTY_ARRAY_9
            imu_msg.orientation_covariance[0] = -1.0
            
            # add header
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame


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
