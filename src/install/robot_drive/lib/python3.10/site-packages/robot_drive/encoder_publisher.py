'''
Saketh Ayyagari
Publishes encoder speeds of each wheel
'''
import sys

import rclpy as ros2
from rclpy.node import Node

#########################
# Constants
#########################
LEFT_ENCODER = None
RIGHT_ENCODER = None


class EncoderPublisher(Node):
    def __init__(self):
        super().__init__("encoder_publisher")
        # initializes node as a publisher
        self.publisher = self.create_publisher(___, 'ticks', 10)
        # how much time between sending messages
        self.timer_period = 1/60
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
    '''
    Called [timer_period] times a second
    '''
    def timer_callback(self):
        pass

def main(args=None):
    ros2.init(args=args)
    encoder_publisher = EncoderPublisher()
    try:
        ros2.spin(encoder_publisher)
    except KeyboardInterrupt:
        pass
    encoder_publisher.destroy_node()
    ros2.try_shutdown()

if __name__ == '__main__':
    main()
