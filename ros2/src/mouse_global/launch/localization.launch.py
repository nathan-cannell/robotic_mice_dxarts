import rclpy

from mouse_global.mouse_utils import get_mouse_namespace

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remapping = [('/tf', 'tf'),
                 ('/tf_static', 'tf_static')]

    mouse_namespaces = ['/mouse_1', '/mouse_2']

    # Iterate through namespaces and add the robot localization node to each
    # mouse_namespaces = get_mouse_namespace()
    for namespace in mouse_namespaces:
        print(namespace)
        robot_localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            remappings=remapping,
            namespace=namespace,
            parameters=[
                {'frequency': 50.0},
                {'odom0': 'odometry'},
                {'odom0_config': [True, True, False, 
                                 False, False, False,
                                 False, False, False,
                                 False, False, True, 
                                 False, False, False]},
                {'imu0': 'imu'},
                {'imu0_config': [False, False, False,
                                 True, True, True,
                                 False, False, False,
                                 True, True, True,
                                 True, True, True]}
            ],
        )
        ld.add_action(robot_localization_node)

    return ld