'''
Saketh Ayyagari

Node publishing odometry data (specifically robot's pose, which consists of
x, y, and theta values)
'''

import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from pmw3901 import PMW3901

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Imu

# packages for changing transform
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
'''
Uses IMU readings and publishes new Odometry message
'''
class IMUOdometryPub(Node): 
    def __init__(self):
        super().__init__('imu_odometry_pub') # intiializes node name
        # constants
        
        # initializing pose (x, y, theta), velocity, and time variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.velocity = 0.0

        self.prev_time = 0.0
        # subscribes to /imu topic
        # imu_callback() is run everytime the node receives a message
        self.create_subscription(Imu, 'imu', 
                            self.imu_callback, 10)

        # publishing to an odometry topic
        self.odometry_publisher = self.create_publisher(Odometry, 
                                                        'pose_estimate', 10)
        # moving average variables
        self.WINDOW_SIZE = 3
        self.prev_measures = []

        self.sensor = PMW3901()
    '''
    Calculates moving average 
    '''
    def calculate_moving_average(self, incoming_value):
        if len(self.prev_measures) < self.WINDOW_SIZE:
            self.prev_measures.append(incoming_value)
            return incoming_value
        else:
            average = sum(self.prev_measures)/self.WINDOW_SIZE
            self.prev_measures.pop(0)
            self.prev_measures.append(incoming_value)
            return average
    '''
    Updates change in time since last called
    '''
    def get_delta_time(self):
        current_time = time.perf_counter()
        delta_time = current_time - self.prev_time
        self.prev_time = current_time
                
        return delta_time
    '''
    Updates pose (x, y, theta) global variables given velocity values
    '''
    def update_pose(self, linear_velocity):
        # integrates velocity values and adds them to current parts of pose
        # updates x and y based on component values of velocity
        # self.x += linear_velocity * math.cos(self.theta) * self.get_delta_time() 
        # self.y += linear_velocity * math.sin(self.theta) * self.get_delta_time()
        self.x += self.sensor.get_motion()[1]
        self.y += self.sensor.get_motion()[0]
    '''
    Uses IMU values to update pose estimate
    '''
    def imu_callback(self, message: Imu):
        # gets acceleration and angular velocity
        linear_acceleration = message.linear_acceleration.x
        angular_velocity = message.angular_velocity.z
        
        # using quaternion values from IMU to estimate orientation
        orientation_quat = message.orientation
        # converting from quaternion to euler and getting one component
        # of euler angle for heading
        self.theta = euler_from_quaternion([orientation_quat.x, 
                                            orientation_quat.y, 
                                            orientation_quat.z, 
                                            orientation_quat.w])[2]

        # getting linear velocity estimate by integrating acceleration
        self.velocity += linear_acceleration * self.get_delta_time() 
       
        self.update_pose(self.velocity)

        # creating new odometry message  
        odom_message = Odometry()
        odom_message.child_frame_id = "base_link"
        odom_message.header.frame_id = "odom"
        odom_message.header.stamp = self.get_clock().now().to_msg()

        # assigning pose values to odometry message
        odom_message.pose.pose.position.x = self.x
        odom_message.pose.pose.position.y = self.y
        odom_message.pose.pose.position.z = 0.0

        
        odom_message.pose.pose.orientation = Quaternion(x=0.0, 
                                                        y=0.0, 
                                                        z=orientation_quat.z, 
                                                        w=orientation_quat.w)

        # assigning velocity values to odometry message
        odom_message.twist.twist.linear.x = self.velocity * math.cos(self.theta)
        odom_message.twist.twist.linear.y = self.velocity * math.sin(self.theta)
        odom_message.twist.twist.angular.z = angular_velocity

        self.get_logger().info(f"Position: ({(self.x, self.y)})")        
        # publishes odometry message to /robot_odometry topic
        self.odometry_publisher.publish(odom_message)

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = IMUOdometryPub()
    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        pass
    odometry_publisher.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
