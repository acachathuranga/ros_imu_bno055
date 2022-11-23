#!/usr/bin/env python3

"""
Author : Achala Athukorala 
"""

import rclpy
from rclpy.node import Node
import traceback
import serial
import time
import adafruit_bno055
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import MagneticField

class SensorIMU(Node):

    def __init__(self):

        # Init node
        super().__init__('ros_imu_bno055_node')

        # Get ros params
        self.get_ros_params()

        # Create an IMU instance
        self.uart = serial.Serial(self.serial_port)
        self.sensor = adafruit_bno055.BNO055_UART(self.uart)

        # Create topics
        self.pub_imu_data = self.create_publisher(Imu, 'imu/data', 10)
        self.pub_imu_magnetometer = self.create_publisher(MagneticField, 'imu/magnetometer', 10)
        self.pub_imu_temperature = self.create_publisher(Temperature, 'imu/temperature', 10)

        # Print node status
        self.get_logger().info(self.get_name() + " ready!")

        # Set frequency
        timer_period = 1.0 / self.frequency # Seconds
        
        # Create program timer
        self.timer = self.create_timer(timer_period, self.run)

    def get_ros_params(self):
        
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('frequency', 20.0)

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value


    def publish_imu_data(self):
            
        imu_data = Imu()  
        
        quaternion = self.sensor.quaternion
        linear_acceleration = self.sensor.linear_acceleration
        gyroscope = self.sensor.gyro
        
        imu_data.header.stamp = self.get_clock().now().to_msg()
        imu_data.header.frame_id = self.frame_id
        
        imu_data.orientation.w = quaternion[0]
        imu_data.orientation.x = quaternion[1]
        imu_data.orientation.y = quaternion[2]
        imu_data.orientation.z = quaternion[3]

        imu_data.linear_acceleration.x = linear_acceleration[0]
        imu_data.linear_acceleration.y = linear_acceleration[1]
        imu_data.linear_acceleration.z = linear_acceleration[2]

        imu_data.angular_velocity.x = gyroscope[0]
        imu_data.angular_velocity.y = gyroscope[1]
        imu_data.angular_velocity.z = gyroscope[2]

        #imu_data.orientation_covariance[0] = -1
        #imu_data.linear_acceleration_covariance[0] = -1
        #imu_data.angular_velocity_covariance[0] = -1

        self.pub_imu_data.publish(imu_data)


    def publish_imu_magnetometer(self):

        imu_magnetometer = MagneticField()

        magnetometer = self.sensor.magnetic

        imu_magnetometer.header.stamp = self.get_clock().now().to_msg()
        imu_magnetometer.header.frame_id = self.frame_id

        imu_magnetometer.magnetic_field.x = magnetometer[0]
        imu_magnetometer.magnetic_field.y = magnetometer[1]
        imu_magnetometer.magnetic_field.z = magnetometer[2]

        self.pub_imu_magnetometer.publish(imu_magnetometer)

    def publish_imu_temperature(self):

        imu_temperature = Temperature()

        temperature = float(self.sensor.temperature)

        imu_temperature.header.stamp = self.get_clock().now().to_msg()
        imu_temperature.header.frame_id = self.frame_id

        imu_temperature.temperature = temperature 

        self.pub_imu_temperature.publish(imu_temperature)

    def run(self):
        try:
            # Publish imu data
            self.publish_imu_data()

            # Publish magnetometer data
            self.publish_imu_magnetometer()

            # Publish temperature data                
            self.publish_imu_temperature()
        except Exception as ex:
            self.get_logger().debug("Exception: %s"%str(ex))
                
def main(args = None):
    rclpy.init(args=args)
    
    imu = None
    try:
        imu = SensorIMU()
        rclpy.spin(imu)
    except Exception as ex:
        print("Exception: %s"%str(ex))
        print(traceback.format_exc())
        
        # Time delay before exiting program (For potential respawn actions)
        time.sleep(3)
        
    imu.uart.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    


