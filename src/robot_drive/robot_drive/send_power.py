import sys

import rclpy as ros2
from rclpy.node import Node
from sensor_msgs.msg import Joy
'''
Saketh Ayyagari
Gets joy message and sends power to robot
'''

class SendPower(Node): 
   def __init__(self):
      # initializes node name
      super().__init__("send_power")
      # subscribes to /cmd_vel topic
      self.subscription = self.create_subscription(Joy, 
      'cmd_vel', self.power_callback, 10) 
   '''
   Sends power to motor driver to control wheels
   '''
   def power_callback(self, message: Joy):
      self.get_logger().info(f"Drive value: {message.axes[0]}, Turn value: {message.axes[1]}")
      # # gets joystick values
      # drive, turn = message.axes

      # # calculates power given joystick values
      # leftPower = drive + turn
      # rightPower = drive - turn


def main(args=None):
   ros2.init(args=args)
   power_node = SendPower()

   ros2.spin(power_node)

   power_node.destroy_node()
   ros2.shutdown()
   
if __name__ == '__main__':
   main()