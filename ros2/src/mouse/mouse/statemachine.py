# 
# @file: statemachine.py
# @author: Martin Li
# @breif: A state machine that switches between AUTONOMOUS_WANDERING, SEEK_DARK, and HIDING states.
# based on the sensor data from presence_mux and other assuming topics. 
# @version: 0.5 (without actual state changing method but set to manual now)
# @date: 2024-05-06

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import select
import tty
import termios

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self.current_state = 'AUTONOMOUS_WANDERING'

        # Initialize presence_mux and all_robots_found_dark to False
        #presence mux will turn true when the state should be switched to SEEK_DARK
        self.presence_mux = False
        self.all_robots_found_dark = False
        
        # Create subscribers for presence_mux and all_robots_found_dark
        self.presence_mux_subscriber = self.create_subscription(
            Bool,
            'presence_mux',
            self.presence_mux_callback,
            10)
        # self.all_robots_found_dark_subscriber = self.create_subscription(
        #     Bool,
        #     'all_robots_found_dark',
        #     self.all_robots_found_dark_callback,
        #     10)

        self.timer = self.create_timer(1.0, self.state_machine)

        # Set terminal to raw mode to read single key presses
        self.old_termios_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def presence_mux_callback(self, msg):
        self.presence_mux = msg.data

    # def all_robots_found_dark_callback(self, msg):
    #     self.all_robots_found_dark = msg.data

    def state_machine(self):
        if self.is_key_pressed('a'):
            self.switch_state()
        else:
            if self.current_state == 'AUTONOMOUS_WANDERING':
                self.autonomous_wandering()
            elif self.current_state == 'SEEK_DARK':
                self.seek_dark()
            elif self.current_state == 'HIDING':
                self.hiding()

    def switch_state(self):
        if self.current_state == 'AUTONOMOUS_WANDERING':
            self.current_state = 'SEEK_DARK'
            self.get_logger().info('Switching to State: SEEK_DARK')
        elif self.current_state == 'SEEK_DARK':
            self.current_state = 'HIDING'
            self.get_logger().info('Switching to State: HIDING')
        elif self.current_state == 'HIDING':
            self.current_state = 'AUTONOMOUS_WANDERING'
            self.get_logger().info('Switching to State: AUTONOMOUS_WANDERING')

    def autonomous_wandering(self):
        self.get_logger().info('State: AUTONOMOUS_WANDERING')
        if self.presence_mux:
            self.current_state = 'SEEK_DARK'

    def seek_dark(self):
        self.get_logger().info('State: SEEK_DARK')
        if self.all_robots_found_dark:
            self.current_state = 'HIDING'

    def hiding(self):
        self.get_logger().info('State: HIDING')

    def is_key_pressed(self, key):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            input_key = sys.stdin.read(1)
            return input_key == key
        return False

    def destroy_node(self):
        super().destroy_node()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_termios_settings)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
