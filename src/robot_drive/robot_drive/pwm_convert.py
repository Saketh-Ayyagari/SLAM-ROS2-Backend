import sys
sys.path.insert(1, '../')
from robot_drive.robot_utils import *

import rclpy as ros2
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
'''
Saketh Ayyagari
Converts joystick values to PWM values that the motor driver can read.
'''
############
# Constants
############
MAX_FREQ = 15000.0
DEAD_ZONE = 0.04 # minimum value joystick needs to be to power motors. Motor power will be set to 0 if joystick value is less than this value

class PWMConvert(Node): 
   def __init__(self):
      # initializes node name
      super().__init__("pwm_conversion")
      # subscribes to /joy topic
      self.subscription = self.create_subscription(Joy, 'joy', self.power_callback, 10)

      # initializes node as a publisher to send to
      # "/cmd_vel" topic
      self.publisher = self.create_publisher(AckermannDriveStamped, 'cmd_vel', 10) 
   '''
   Converts joystick values to PWM values that the motor driver can read
   '''
   def power_callback(self, message: Joy):
      # gets joystick values
      drive_joystick = message.axes[0]
      turn_joystick = message.axes[1]

      # checks if joystick values are above dead zone value
      # else sets them to 0
      if drive_joystick < DEAD_ZONE or drive_joystick > -DEAD_ZONE:
         drive_joystick = 0
      if turn_joystick < DEAD_ZONE or turn_joystick > -DEAD_ZONE:
         turn_joystick = 0
      

      # putting information into messages
      pwm_message = AckermannDriveStamped() # first index is 'drive' value, second index is 'turn' value

      pwm_message.drive.speed = remap_range(drive_joystick, -1, 1, 
      -MAX_FREQ, MAX_FREQ)

      pwm_message.drive.steering_angle = remap_range(turn_joystick, -1, 1, 
      -MAX_FREQ, MAX_FREQ)

      # sends pwm values to the /pwm topic
      self.publisher.publish(pwm_message)
      
def main(args=None):
   ros2.init(args=args)
   pwm_conversion_node = PWMConvert()

   ros2.spin(pwm_conversion_node)

   pwm_conversion_node.destroy_node()
   ros2.shutdown()
   
if __name__ == '__main__':
   main()