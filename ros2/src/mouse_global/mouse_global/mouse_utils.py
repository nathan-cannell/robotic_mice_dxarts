import rclpy
from rclpy.node import Node

from ros2node.api import get_node_names 
from ros2topic.api import get_topic_names

# Get the namespaces of current open nodes
def get_mouse_namespace(node):
    mouse_nodes = [mouse_node.namespace for mouse_node in get_node_names(node=node) if mouse_node.namespace.startswith('/mouse_')]

    return mouse_nodes

# Returns a topic from the list of mouse namespaces
def get_mouse_topics(node, filter):
    mouse_topics = [topic for topic in get_topic_names(node=node) if topic.startswith('/mouse_') and topic.endswith(filter)]

    return mouse_topics