# 
# @file: fuel_mux.py
# @author: Ethan Widger
# @breif: A subscriber to all /mouse_X/fuel topics.
# @version: 0.1
# @date: 2024-05-06
#
from std_msgs.msg import Float32

import time
import rclpy
from rclpy.node import Node
from mouse_global.mouse_utils import get_mouse_topics
from functools import partial

class FuelMUX(Node):
    fuel = dict()

    # Initialize the node
    def __init__(self):
        super().__init__('mouse_global')
        self.get_logger().info('Fuel MUX started...')

        self.declare_parameter('batt_th', 30.0)

        timer_period = 1.0 # s
        self.timers_ = self.create_timer(timer_period, self.timer_callback)

        # Need to wait before topics can be seen
        time.sleep(0.5)
        self.connect()
    
    # Intitialize the connections to ROS
    def connect(self):
        # Find mouse bot topics to follow
        mouse_topics = get_mouse_topics(self, "fuel")
        for topic in mouse_topics:
            if (topic not in self.fuel):
                self.fuel[topic] = -1
                self.subscribers_ = self.create_subscription(Float32, topic, partial(self.topic_callback, topic=topic), 10)

    # Callback for the timer to report to the user
    def timer_callback(self):
        threshhold = self.get_parameter('batt_th').get_parameter_value().double_value

        # Check if there is a new topic to subscribe to.
        if (len(get_mouse_topics(self, "fuel")) > len(self.fuel)):
            self.connect()
            return
    
        # Check if there are any topics
        if (len(self.fuel < 1)):
            self.get_logger().warn('No topics published.')
            return

        min_mouse = min(self.fuel, key=self.fuel.get) 
        if self.fuel[min_mouse] < threshhold:
            self.get_logger().warn('%s getting critically low at %d.' % (min_mouse, self.fuel[min_mouse]))
        else:
            self.get_logger().info('%s lowest at %d.' % (min_mouse, self.fuel[min_mouse]))
    
    # Callback for the topic listener and set the values in the dictionary
    def topic_callback(self, msg, topic):
        self.fuel[topic] = msg.data


def main(args=None):
    rclpy.init(args=args)

    fuel_mux = FuelMUX()

    rclpy.spin(fuel_mux)

    rclpy.shutdown()


if __name__ == '__main__':
    main()