import sys

import rclpy as ros2
from rclpy.node import Node
from sensor_msgs.msg import Joy
'''
Saketh Ayyagari
Converts joystick values to PWM values that the motor driver can read.
'''

class ThrottleNode(Node): 
   def __init__(self):
      # initializes node name
      super().__init__("throttle_node")
      # subscribes to /cmd_vel topic
      self.subscription = self.create_subscription(Joy, 
      'cmd_vel', self.power_callback, 10) 
   '''
   Converts joystick values to PWM values that the motor driver can read
   '''
   def power_callback(self, message: Joy):
      pass

def main(args=None):
   ros2.init(args=args)
   power_node = ThrottleNode()

   ros2.spin(power_node)

   power_node.destroy_node()
   ros2.shutdown()
   
if __name__ == '__main__':
   main()