import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random


class Mouse(Node):
    # Create mouse object
    def __init__(self):
        super().__init__('mouse')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5,self.wander) # publishes twist to cmd_vel every .5 seconds
    
    # wandering function
    def wander(self):
        
        
        twist = Twist()
        twist.linear.x=0.2 # move forward with a linear velocity 0.2 m/s

        # CURRENT PROBLEM: ITS TOO RANDOM
        twist.angular.z = random.uniform(-1.0,1.0) # random angular velocity between -1 and 1 radians/s
        self.publisher_.publish(twist)
        
        # prints the linear and angular velocity
        print(twist.linear.x,twist.angular.z)



def main(args=None):
    rclpy.init(args=args)
    mouse = Mouse()
    
    # begins the mouse wandering
    # ENDS WHEN CNTRL+C         
    while rclpy.ok():
        rclpy.spin_once(mouse)
        mouse.wander() # run wandering
        print() # new line

    mouse.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


