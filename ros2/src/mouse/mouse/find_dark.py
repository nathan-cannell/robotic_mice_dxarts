import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Presence
from sensor_msgs.msg import Illuminance
from typing import NamedTuple
import random
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

# Python timers resource:
# https://ioflood.com/blog/python-wait/

MIN_RANGE = 0.1524 # half a foot. 
DARK_FOUND = 2500
# Illuminance message:
# https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Illuminance.html
class IlluminanceDataStruct(NamedTuple):
    time_stamp: float
    illuminance: float
    variance: float

class PresenceStruct(NamedTuple):
    time_stamp: float
    presenceDetected: int

class DarkFinder(Node):

    def __init__(self):
        # true means illuminance decreased, false means that it increased.
        self.illuminance_changes_booleans = []
        self.previous_angular_z = []

        # Making a subscriber for the light_sensor:
        self.lightSensorSubscriber = self.create_subscription(
            msg_type=Illuminance, 
            topic='/mouse_1/dark',
            callback=self.light_sensor_callback,
            qos_profile=20,
        )
        
        # Making a subscriber for the presence sensor:
        self.presenceSensorSubscriber = self.create_subscription(
            msg_type = Presence, 
            topic='mouse_1/presence',
            callback=self.presence_sensor_callback,
            qos_profile=20,
        )

        # Making a subscriber for the range sensor:
        self.subscription = self.create_subscription(
             msg_type=Range,
             topic='/mouse_3/range', callback=self.range_sensor_callback,
             qos_profile=20)

        # making the publisher:
        # Twist definition: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        self.movementPublisher = self.create_publisher(
             msg_type=Twist, topic='/mouse_1/cmd_vel', qos_profile=10)
        
        timer_period = 0.50  # seconds
        self.intermediary_timer = self.create_timer(timer_period, self.intermediary_timer_callback)

        # Initializing our lists:
        self.illuminanceDataList = []
        self.presenceDataList = []

        # https://www.pranaair.com/us/blog/illuminance-levels-indoors-the-standard-lux-levels/
        self.dusky_illuminance_cutoff = 10
        self.dark_illuminance_cutoff = 2

        self.range_reading = 0
        self.finished_obstacle_check = True
        self.finished_current_movement = True

    def range_sensor_callback(self, range_message):
         # https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Range.html
         self.range_reading = range_message.range

    def light_sensor_callback(self, light_sensor_message):
         # Pulling in data from the lightSensorSubscriber's latest message.
         self.get_logger().info("Start of the light sensor callback")
         timestamp:float = light_sensor_message.header.stamp.sec + light_sensor_message.header.stamp.nanosec * 1e-9
         illuminance:float = light_sensor_message.illuminance
         variance:float = light_sensor_message.variance
         illuminance_data = IlluminanceDataStruct(timestamp, illuminance, variance)
         self.illuminanceDataList.append(illuminance_data)

    def presence_sensor_callback(self, presence_sensor_message):
        # Pulling in data from the presenceSensorSubscriber's latest message.
        self.get_logger().info("Start of the presence sensor callback")
        timestamp:float = presence_sensor_message.header.stamp.sec + presence_sensor_message.header.stamp.nanosec * 1e-9
        presenceDetected:int = presence_sensor_message.presenceDetected
        presence_data = PresenceStruct(timestamp, presenceDetected)
        self.presenceDataList.append(presence_data)

    def check_for_obstacles(self):
        self.finished_obstacle_check = False
        # Rotate one way:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.3
        self.movementPublisher.publish(twist)
        time.sleep(0.2)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.3
        self.movementPublisher.publish(twist)
        time.sleep(0.2)
        # Rotate the other way
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.3
        self.movementPublisher.publish(twist)
        time.sleep(0.2)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -0.3
        time.sleep(0.2)
        self.finished_obstacle_check = True 

    def intermediary_timer_callback(self):
        # If we finished the last movement, we can now execute the next one:
        if self.finished_obstacle_check and self.finished_current_movement:
            self.timer_callback()

    def timer_callback(self):
        self.get_logger().info('start of timer_callback')
        self.finished_current_movement = False

        if self.illuminanceDataList[len(self.illuminanceDataList) - 1] >= DARK_FOUND:
             # We have found the dark, won't move any more. 
             twist = Twist()
             twist.linear.x = 0
             twist.angular.z = 0
             self.movementPublisher.publish(twist)
             self.finished_current_movement = True

        # Otherwise, we have not found the dark yet, keep looking: 
        else: 
            if self.range_message > MIN_RANGE:
                # Turn in a random direction to avoid the obstacle
                twist.linear.x = 0
                twist.angular.z = random.choice([1.6, -1.6, 2.1, -2.1, 2.4, -2.4])
                time.sleep(0.2)
                self.finished_current_movement = True
                
            elif len(self.illuminanceDataList) > 1:
                self.check_for_obstacles()
                
                if self.finished_obstacle_check:
                    most_recent_val = self.illuminanceDataList[len(self.illuminanceDataList) - 1]
                    previous_val = self.illuminanceDataList[len(self.illuminanceDataList) - 2]
                    if most_recent_val > self.dark_illuminance_cutoff:
                        if (most_recent_val <= previous_val):
                            self.get_logger().info('case 1: illumination decreased or stayed the same')
                            # Continue in the current direction (going forwards):
                            twist = Twist()
                            twist.linear.x = 0.3
                            twist.angular.z = 0
                            self.illuminance_changes_booleans.append(True)
                            self.movementPublisher.publish(twist)
                            time.sleep(0.2)
                            self.finished_current_movement = True

                        else:
                            self.get_logger().info('case 2: illumination increased')
                            # TODO:
                            # Move while turning. 
                            twist.linear.x = 0.2
                            twist.angular.z = random.choice([0.17, -0.17, -0.34, 0.34, 0.69, -0.69, 1.04, -1.04, 1.39, -1.39])
                            self.illuminance_changes_booleans.append(False)
                            self.movementPublisher.publish(twist)   
                            time.sleep(0.2)
                            self.finished_current_movement = True       

def main(args=None):
        rclpy.init(args=args)

        movementPublisher = DarkFinder()
        movementPublisher.get_logger().info('returned from call to DarkFinder')
        
        # Keep program executing until this node has stopped.
        rclpy.spin(movementPublisher)

if __name__ == '__main__':
        main()


