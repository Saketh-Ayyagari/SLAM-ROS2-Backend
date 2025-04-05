import sys
import pygame as game

import rclpy as ros2
from rclpy.node import Node
from sensor_msgs.msg import Joy
'''
Saketh Ayyagari
Gets joystick values from controller using Pygame and processes them in Joy message.
Publishes them to /cmd_vel topic
'''
############
# Constants
############
GAMEPAD_LEFT_Y_AXIS=1
GAMEPAD_RIGHT_X_AXIS=3

class JoyNode(Node): 
   def __init__(self):
      super().__init__('joy_node')
      # initializes node as a publisher
      self.publisher = self.create_publisher(Joy, 'joy', 10)
      # how much time between sending messages
      self.timer_period = 1/60
      self.timer = self.create_timer(self.timer_period, self.controller_callback)
      # initializing pygame and joystick
      game.init()
      
      try: # initializes the joystick object if Joystick object is present.
         # gets the joystick object and initializes it
         self.joystick = game.joystick.Joystick(0) 
         self.joystick.init()
      except: # throws error if Joystick object cannot be created (controller is not plugged in) 
         print("Error: Controller not connected!")
      
      # y = vertical, x = horizontal
      # range of each joystick is [-1, 1]
      self.left_y = 0
      self.right_x = 0
   '''
   Gets the joystick values of the left y-axis and the right x-axis
   '''
   def get_joystick_values(self):
      return (-self.joystick.get_axis(GAMEPAD_LEFT_Y_AXIS), 
              self.joystick.get_axis(GAMEPAD_RIGHT_X_AXIS))
   '''
   Uses joystick values and publishes them as a Joy message
   '''
   def controller_callback(self):
      game.event.get()

      # gets the position of each axis of a joystick controller
      self.left_y, self.right_x = self.get_joystick_values()

      # processing joystick values in a Joy message
      joystick_message = Joy()
      joystick_message.axes = [self.left_y, self.right_x]

      # publishing joystick message
      self.publisher.publish(joystick_message)

def main(args=None):
   ros2.init(args=args)
   joy_node = JoyNode()
   
   ros2.spin(joy_node)
   # un-initializes joysticks after use
   game.joystick.quit()
   joy_node.destroy_node()
   ros2.shutdown()

if __name__ == '__main__':
   main()