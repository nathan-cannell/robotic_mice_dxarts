import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestDistance(Node):
    TIMER_PERIOD = 5
    stop = False
    once = True

    # Initialize the node
    def __init__(self):
        super().__init__('test')
        self.get_logger().info('Test Distance initiated...')

        self.create_timer(self.TIMER_PERIOD, self.timer_callback)
        self.publishers_ = self.create_publisher(Twist, '/mouse_1/cmd_vel', 10)

    def timer_callback(self):
        if not self.once:
            return
        
        twist_msg = Twist()
        if self.stop == False:
            # Not moving forward, only rotating.
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.2
            self.stop = True
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.once = False
        
        self.publishers_.publish(twist_msg)
        self.get_logger().info('Speed set: %f' % (twist_msg.linear.x))

def main(args=None):
    rclpy.init(args=args)

    test = TestDistance()
    for i in range(2):
        test.get_logger().info('Spin %d' % (i))
        rclpy.spin_once(test)

    rclpy.shutdown()

if __name__ == '__main__':
    main()