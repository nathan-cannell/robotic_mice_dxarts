import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from custom_msgs.msg import Encoders
from sensor_msgs.msg import Imu 
from typing import NamedTuple
from nav_msgs.msg import Odometry
import std_msgs.msg
import math

# Publisher/subscriber resources:
# https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
# http://wiki.ros.org/ROS/Tutorials/CustomMessagePublisherSubscriber%28python%29
# https://ros2-tutorial.readthedocs.io/en/latest/publishers_and_subscribers.html

# Going to use a struct to store data from the subscriber:
# Structs in python: https://stackoverflow.com/questions/35988/c-like-structures-in-python
class MotorOdometryStruct(NamedTuple):
    time_stamp: float
    right_motor_counts:int
    left_motor_counts:int
    right_position: float # distance traveled for the right wheel
    left_position: float # distance traveled for the left wheel
    right_vel: float # units are in meters/s
    left_vel: float # units are in meters/s
    wheel_radius: float
    wheel_separation: float

# https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Quaternion.html
class Vector4(NamedTuple):
    x_value: float
    y_value: float
    z_value: float
    w_value: float

class Vector3(NamedTuple):
    x_value: float
    y_value: float
    z_value: float

class OdometryInfo(Node):

    def __init__(self):
        super().__init__('mouse')
        self.get_logger().info("start of init")

        self.wheel_separation = 0.074
        self.wheel_radius = 0.0167

        # The conversion value: (revolutions per minute -> radians per second)
        self.rpm_to_radians = 0.10471975512

        # Varaibles needed for accumulate:
        self.sum_ = 0
        self.buffer_ = []
        self.next_insert_ = 0
        self.buffer_filled:bool = False

        # assuning that we start from no movement: 
        self.previousLinearAcceleration = Vector3(0, 0, 0)
        self.previousTime = self.get_clock().now().to_msg()

        # Odometry definition: https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
        self.odometryPublisher = self.create_publisher(
            msg_type=Odometry, topic='/mouse_1/odometry', qos_profile=10)
        self.get_logger().info("Made the odometry publisher")

        # Initializing a python dictionary to store the messages for the encoders and imu
        self.encoders_received_values:dict = {}
        #self.imu_received_values:dict = {}

        # making the subscriber for the motor encoders (to read data from '/mouse_1/encoders/')
        self.encodersSubscriber_ = self.create_subscription(
            msg_type=Encoders, 
            topic='/mouse_1/encoders', 
            callback=self.encoders_callback, 
            qos_profile=20
        )
        self.get_logger().info('made the encodersSubscriber')

        timer_period = 0.20  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.encoders_most_recent_key = -1
        #self.imu_most_recent_key = -1

        # variables for the odometry message:
        self.odometrySequenceNumber:int = 0

        # Assuming that the initial position is the origin:
        self.pos_x:float = 0.0
        self.pos_y:float = 0.0
        self.theta:float = 0.0

        # encoder information: 
        self.counts_per_rev:int = 28

    # This should be called whenever we receive a message:
    def encoders_callback(self, encoders_msg):
        self.get_logger().info("We have entered the encoders callback")
        # See the header definition: https://docs.ros.org/en/melodic/api/std_msgs/html/msg/Header.html
        timestamp:float = encoders_msg.header.stamp.sec + encoders_msg.header.stamp.nanosec * 1e-9
        right_motor_counts:int = encoders_msg.right_motor_counts
        left_motor_counts:int = encoders_msg.left_motor_counts
        right_pos:float = encoders_msg.right_position
        left_pos:float = encoders_msg.left_position
        right_vel:float = encoders_msg.right_vel
        left_vel:float = encoders_msg.left_vel
        radius:float = self.wheel_radius
        separation_between_wheels:float = self.wheel_separation
        #self.get_logger().info("finished reading values from encoders_msg")

        # Updating the key/index: 
        self.encoders_most_recent_key =  self.encoders_most_recent_key + 1
        #self.get_logger().info('Updated the key')

        # Putting the data into our struct:
        encoders_data = MotorOdometryStruct(timestamp, right_motor_counts, left_motor_counts, right_pos, left_pos, right_vel, left_vel, radius, separation_between_wheels)

        self.encoders_received_values[self.encoders_most_recent_key] = encoders_data
        self.get_logger().info('len(self.encoders_received_values) is: %d' % len(self.encoders_received_values))

    def timer_callback(self):
        self.get_logger().info("We have entered the timer_callback ")
        # Processing our data (forming Odometry message): comparing the last two entries
        self.get_logger().info('len(self.encoders_received_values) is: ' + str(len(self.encoders_received_values)))
        if (len(self.encoders_received_values) > 1):
            current_encoder_data = self.encoders_received_values.get(self.encoders_most_recent_key)
            previous_encoder_data = self.encoders_received_values.get(self.encoders_most_recent_key - 1)
            
            # Getting distance traveled:
            right_distance = current_encoder_data.right_position
            self.get_logger().info('right_distance is: %f' % right_distance)
            left_distance = current_encoder_data.left_position
            self.get_logger().info('left is: %f' % left_distance)

            # Change in orientation:
            # TODO: check that delta_theta is correct. 
            delta_theta = (right_distance - left_distance) / self.wheel_separation
            self.get_logger().info('delta_theta is: %f' % delta_theta)

            # First half of equation: change in position (divide by 2.0 to get the midpoint between the two wheels -> straight line movement estimate)
            delta_x = (left_distance + right_distance) / 2.0 * math.cos(delta_theta) # Cosine of change in angle gets the adjacent side (x position)
            self.get_logger().info('delta_x is: %f' % delta_x)
            delta_y = (left_distance + right_distance) / 2.0 * math.sin(delta_theta) # Sine of the change in orientation gets the opposite side of the triangle (the y position)
            self.get_logger().info('delta_y is: %f' % delta_y)

            # Updating our position:
            self.pos_x  = self.pos_x + delta_x
            self.get_logger().info('pos_x is: %f' % self.pos_x)
            self.pos_y = self.pos_y + delta_y
            self.get_logger().info('pos_y is: %f' % self.pos_y)
            # Updating our orientation: 
            self.theta = self.theta + delta_theta

            # creating the header for the Odometry message:
            # https://robotics.stackexchange.com/questions/50135/what-is-the-proper-way-to-create-a-header-with-python
            # https://stackoverflow.com/questions/74976911/create-an-odometry-publisher-node-in-python-ros2
            odom = Odometry()
            odom.header = std_msgs.msg.Header()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            # generating the pose for the odometry message:
            odom.pose.pose.position.x = self.pos_x
            odom.pose.pose.position.y = self.pos_y
            odom.pose.pose.position.z = float(0.0)

            # Uses a quaternion as an orientation
            # See header definition: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Quaternion.html

            # Z in the quaternion -> imaginary component, defines the axis of rotation
            odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
            # W in the quaterian -> measurement of rotation, the real component
            odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

            # Start of linear_velocity calculations:
            delta_time = current_encoder_data.time_stamp - previous_encoder_data.time_stamp

            # linear velocity components: (x is horizontal movement, y is forward movement)
            # Total linear velocity should be the average of both wheel values
            linear_velocity_estimate_total = (current_encoder_data.right_vel + current_encoder_data.left_vel) / 2
            linear_velocity_z = float(0.0)

            # https://www.khanacademy.org/science/physics/two-dimensional-motion/two-dimensional-projectile-mot/a/what-are-velocity-components
            sin_theta = math.sin(self.theta)
            linear_velocity_y = sin_theta * linear_velocity_estimate_total
            cos_theta = math.cos(self.theta)
            linear_velocity_x = cos_theta * linear_velocity_estimate_total
            self.get_logger().info('new linear_velocity_x using theta/triganometry is: %f' % linear_velocity_x)
            self.get_logger().info('new linear_velocity_y using theta/triganometry is: %f' % linear_velocity_y)

            odom.twist.twist.linear.x = float(linear_velocity_x)
            odom.twist.twist.linear.y = float(linear_velocity_y)
            odom.twist.twist.linear.z = float(linear_velocity_z)

            # Getting the angular velocities:
            # https://automaticaddison.com/how-to-calculate-the-velocity-of-a-dc-motor-with-encoder/

            rpm_right = current_encoder_data.right_motor_counts * 60 / self.counts_per_rev
            rpm_left = current_encoder_data.left_motor_counts * 60 / self.counts_per_rev
            ang_velocity_right = rpm_right * self.rpm_to_radians
            ang_velocity_left = rpm_left * self.rpm_to_radians
            ang_velocity_z = (ang_velocity_right + ang_velocity_left) / 2

            # the x and y components should be zero as we have two-dimensional motion. 
            ang_velocity_x = float(0.0)
            ang_velocity_y = float(0.0)

            # angular velocity = change in angle / change in time. 
            # Resource: https://medium.com/@nahmed3536/wheel-odometry-model-for-differential-drive-robotics-91b85a012299
            # Supposing that we estimate the distance traveled as a straight line path,
            # the below angle would represent the change in the robot's position
            # if we formed a triangle from the starting and ending points.
            delta_alpha = delta_theta / 2; 
            # And angular velocity is the change in angular rotation over time.
            ang_velocity_z = float(delta_alpha / delta_time)
            self.get_logger().info('theta: %f' % self.theta)
            self.get_logger().info('delta_theta: %f' % delta_theta)
            self.get_logger().info('delta_alpha: %f' % delta_alpha)
            self.get_logger().info('angular velocity: %f' % ang_velocity_z)
            
            odom.twist.twist.angular.x = float(ang_velocity_x)
            odom.twist.twist.angular.y = float(ang_velocity_y)
            odom.twist.twist.angular.z = float(ang_velocity_z)

            self.odometrySequenceNumber = self.odometrySequenceNumber + 1
            self.get_logger().info("Publishing the odom now: ")

            # Publishing the data:
            self.odometryPublisher.publish(odom)      

def main(args=None):
        rclpy.init(args=args)

        odometryPublisher = OdometryInfo()
        odometryPublisher.get_logger().info('returned from call to OdometryInfo')
        
        # Keep program executing until this node has stopped.
        rclpy.spin(odometryPublisher)

if __name__ == '__main__':
        main()