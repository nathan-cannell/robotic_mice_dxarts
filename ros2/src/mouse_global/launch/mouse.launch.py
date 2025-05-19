from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    mouse_id_service = Node(
        package='mouse_global',
        executable='id_service',
        name='id_service',
    )
    ld.add_action(mouse_id_service)

    presence_mux = Node(
        package='mouse_global',
        executable='presence_mux',
        name='presence',
    )
    ld.add_action(presence_mux)

    fuel_mux = Node(
        package='mouse_global',
        executable='fuel_monitor',
        name='fuel_monitor',
    )
    ld.add_action(fuel_mux)

    return ld