# 
# @file: presence_mux.py
# @author: Ethan Widger
# @breif: A subscriber to all /mouse_X/presence topics and publishes /presence_mux topic.
# @version: 0.1
# @date: 2024-05-06
#
from custom_msgs.msg import Presence
from std_msgs.msg import Bool

import time
import rclpy
from rclpy.node import Node
from mouse_global.mouse_utils import get_mouse_topics
from functools import partial

class PresenceMUX(Node):
    presence = dict()

    # Initialize the node
    def __init__(self):
        super().__init__('mouse')
        self.get_logger().info('Presence MUX started...')

        self.publishers_ = self.create_publisher(Bool, 'presence_mux', 10)

        timer_period = 1.0 # s
        self.timers_ = self.create_timer(timer_period, self.timer_callback)

        # Need to wait before topics can be seen
        time.sleep(0.5)
        self.connect()
    
    # Intitialize the connections to ROS
    def connect(self):

        # Find mouse bot topics to follow
        mouse_topics = get_mouse_topics(self, "presence")
        for topic in mouse_topics:
            if (topic not in self.presence):
                self.presence[topic] = -1
                self.subscribers_ = self.create_subscription(Presence, topic, partial(self.topic_callback, topic=topic), 10)

        self.get_logger().info('Subscribed to: %s' % (', '.join(mouse_topics)))

        
    # Call back for the publisher timer and check to see the maximum value of all the mouse presence
    def timer_callback(self):
        mux_out = Bool()

        # Check if there is a new topic to subscribe to.
        if (len(get_mouse_topics(self, "presence")) > len(self.presence)):
            self.connect()
            return
        
        # Check if there are any topics
        if (len(self.presence) < 1):
            self.get_logger().warn('No topics published.')
            return

        max_mouse = max(self.presence, key=self.presence.get)
        mux_out.data = self.presence[max_mouse] > 0
        self.publishers_.publish(mux_out)
        self.get_logger().info('Presence: %s %d' % (max_mouse, self.presence[max_mouse]))



    # Callback for the topic listener and set the values in the dictionary
    def topic_callback(self, msg, topic):
        self.presence[topic] = msg.presence



def main(args=None):
    rclpy.init(args=args)

    presence_mux = PresenceMUX()

    rclpy.spin(presence_mux)

    rclpy.shutdown()

if __name__ == '__main__':
    main()