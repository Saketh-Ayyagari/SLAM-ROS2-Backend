import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# packages for changing transform
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from math import pi
'''
Saketh Ayyagari
Takes odometry data and updates base_link transform
'''

class OdometryTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('odometry_to_transform') # initializing node name
        # subscribing to odometry topic
        self.create_subscription(Odometry, 'pose_estimate', 
                            self.odom_callback, 10)

        # initializing transform tools like TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
    '''
    Run every time an odometry message is received
    '''
    def odom_callback(self, odom_message: Odometry):
        # uses odometry data to change location of ROS2 transform
        new_transform = self.get_new_transform(odom_message)
        self.tf_broadcaster.sendTransform(new_transform)
    
    '''
    Updates ROS2 transform's position/orientation using odometry data
    Returns TransformStamped message
    '''
    def get_new_transform(self, message: Odometry):
        tf = TransformStamped()

        # header
        tf.header.stamp = self.get_clock().now().to_msg()

        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        
        # translation
        tf.transform.translation.x = message.pose.pose.position.x
        tf.transform.translation.y = message.pose.pose.position.y
        tf.transform.translation.z = 0.0
        # rotation
        tf.transform.rotation.x = message.pose.pose.orientation.x
        tf.transform.rotation.y = message.pose.pose.orientation.y
        tf.transform.rotation.z = message.pose.pose.orientation.z
        tf.transform.rotation.w = message.pose.pose.orientation.w

        return tf
    
def main(args=None):
    rclpy.init(args=args)
    tf_odom_broadcaster = OdometryTransformBroadcaster()
    try:
        rclpy.spin(tf_odom_broadcaster)
    except KeyboardInterrupt:
        pass
    tf_odom_broadcaster.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
