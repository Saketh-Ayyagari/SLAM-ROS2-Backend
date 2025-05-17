import sys
from robot_drive.robot_utils import *
from robot_drive.pca9685 import *

import rclpy as ros2
from rclpy.node import Node
from geometry_msgs.msg import Twist
'''
Saketh Ayyagari
Gets joy message and sends power to robot
'''

MAX_FREQ = 15000.0

class SendPower(Node): 
   def __init__(self):
      # initializes node name
      super().__init__("send_power")
      # subscribes to /cmd_vel topic
      self.subscription = self.create_subscription(Twist, 
      'cmd_vel', self.power_callback, 10) 

      # initializes pwm motor drive
      self.motor_driver = PCA9685()
   '''
   Sends power to motor driver to control wheels
   '''
   def power_callback(self, message):
      # gets pwm values
      drive = message.linear.y
      turn = message.angular.z

      # sends power to each of the motors
      self.set_drive_turn(drive, turn)
   '''
   Given pwm values, control the robot's drive and turn power
   '''
   def set_drive_turn(self, drive, turn):
      # calculates differential drive power given values from "drive" and "turn"
      leftPWM = clamp(drive + turn, -MAX_FREQ, MAX_FREQ)
      rightPWM = clamp(drive - turn, -MAX_FREQ, MAX_FREQ)
      
      '''
      NOTE: Try to refactor this to make it more readable...
      '''
      # changes pwm signals based on sign of the power
      if leftPWM != 0:
         self.motor_driver.setServoPulse(DC_MOTOR_PWM1, abs(leftPWM)) # for left motor set speed
         if leftPWM > 0:
            # left wheel moves counterclockwise
            self.motor_driver.setServoPulse(DC_MOTOR_INA1,0) # set INA1 L 
            self.motor_driver.setServoPulse(DC_MOTOR_INA2,19999) # set INA2 H
         else:
            # right wheel moves clockwise
            self.motor_driver.setServoPulse(DC_MOTOR_INA1,19999) # set INA1 H 
            self.motor_driver.setServoPulse(DC_MOTOR_INA2,0) # set INA2 L
      else:
         self.motor_driver.setServoPulse(DC_MOTOR_PWM1,0) # for TB6612 stop pwm signal

      
      if rightPWM != 0:
         self.motor_driver.setServoPulse(DC_MOTOR_PWM2, abs(rightPWM)) # for right motor set speed
         if rightPWM > 0:
            # right wheel moves clockwise
            self.motor_driver.setServoPulse(DC_MOTOR_INB1,19999) # set INB1 H 
            self.motor_driver.setServoPulse(DC_MOTOR_INB2,0) # set INB2 L
         else:
            # right wheel moves clockwise
            self.motor_driver.setServoPulse(DC_MOTOR_INB1,0) 
            self.motor_driver.setServoPulse(DC_MOTOR_INB2,19999) 
      else:
         self.motor_driver.setServoPulse(DC_MOTOR_PWM2,0) # for TB6612 stop pwm signal
   
   
def main(args=None):
   ros2.init(args=args)
   power_node = SendPower()

   ros2.spin(power_node)

   power_node.destroy_node()
   ros2.shutdown()
   
if __name__ == '__main__':
   main()
