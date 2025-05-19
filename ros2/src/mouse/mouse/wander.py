# 
# @file: wander.py
# @author: Nathan Cannell
# @breif: wandering controls for mouse movement.
# @version: 3.0
# @date: 2024-04-28
#

import rclpy 
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import math
import random
import time



MIN_RANGE = 0.015
TIMER_PERIOD = 0.5
#range_value = -1
class Mouse(Node):
    range_value = 0

    # Create mouse object
    def __init__(self):
        self.move_timer = 5
        self.stop_timer = 0
        #global range_value
        #self.stop_count = 5
        super().__init__('wander')
        self.publisher_ = self.create_publisher(Twist, '/mouse_1/cmd_vel', 10)
        self.timer = self.create_timer(TIMER_PERIOD,self.wander) # publishes twist to cmd_vel every .5 seconds
        # 'range' is the range topic
        self.subscription = self.create_subscription(Range,'/mouse_1/range', self.range_callback, 10)
        self.subscription  # Prevent unused variable warning


    
    # taken from fuel_mux.py
    # Intitialize the connections to ROS
    '''
    def connect(self):
        # Find mouse bot topics to follow
        mouse_topics = get_mouse_topics(self, "range")
        for topic in mouse_topics:
            if (topic not in self.wander):
                self.presence[topic] = -1
                self.subscribers_ = self.create_subscription(Float32, topic, partial(self.topic_callback, topic=topic), 10)
    '''
    # gets the range value from sensor
    def range_callback(self, msg):
        self.range_value = msg.range
        self.get_logger().info(self.range_value)

    
    # wandering function
    # don't know if msg is the way to go ab it
    def wander(self):
        # access range value directly
        range_val = self.range_value
        print(range_val)
        print("move timer ", self.move_timer)
        print("stop timer ", self.stop_timer)
        twist = Twist()
        # start spinning

        #twist.angular.z = 0.0 # random angular velocity between -1 and 1 radians/s
        #self.publisher_.publish(twist)

        
        turn_range = 0.050 # 50 mm
        print(range_val)
        r = random.random()
        

        
        if(self.move_timer > 0):
            # add or not turn   
            if(range_val > MIN_RANGE): 
                #range_callback(self,range_val)
                twist.linear.x = 0.2
                twist.angular.z = 0.0
                # move forward with a linear velocity 0.2 m/s
                print("range: ", range_val)
            # 50/50 chance to turn left or right
            elif(r < 0.5):
                twist.linear.x = 0.0
                print(r, " < 0.5")
                # turn left until there's nothing within 50mm of the sensor
                if (range_val < turn_range):
                    
                    twist.angular.z = np.pi/4 # pi/4 rad/s
                    print("stuck1")
                
            else:
                print(r, " > .5")
                # turn right until there's nothing within 50
                # cm of the sensor
                twist.linear.x = 0.0
                if (range_val < turn_range):
                    # turn right
                    twist.angular.z = -np.pi/4  # pi/4 rad/s
                    print("stuck2")
            if(self.move_timer <= 1):
                self.stop_timer = random.randint(3,7)/TIMER_PERIOD

            self.publisher_.publish(twist)
            self.move_timer -= 1
            
        # stopped
        elif(self.stop_timer > 0):
            twist.linear.x = 0.0
            twist.angular.z = 0.0


            # RANDOM TURNING
            if(r > 0.9):
                # TURN RIGHT 50% CHANCE
                if(random.random() < 0.5):
                    twist.angular.z = np.pi/4 # turn pi/4 rad/s
                else:
                    twist.angular.z = -np.pi/4 # turn pi/4 rad/s
            # ZERO VELOCITY
            print("range: ", range_val)
            if(self.stop_timer <= 1):
                self.move_timer = random.randint(3,7)/ TIMER_PERIOD
                twist.angular.z = 0.0
            
            self.publisher_.publish(twist)
            self.stop_timer -= 1


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



