# Import the ROS required modules
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import the required ROS interfaces
# TODO

# Import the builtin 'Duration' message
from builtin_interfaces.msg import Duration

import math

# Import this to type the functions
from typing import List

class BehaviorInterface:

    def __init__(self, parent_node: Node):
        """! Initialize the controller server interface.
        @param parent_node "Node" node instance that will be used to initialize
            the ROS-related attributes of class.
        """
        self.node = parent_node
        self.logger = parent_node.get_logger()
        # TODO

    def call_spin_action_client(self, # TODO ):
        # TODO
        pass

    def call_wait_action_client(self, # TODO ):
        # TODO
        pass


def test_behavior_server(args=None):
    
    # Initialize rclpy
    rclpy.init(args=args)

    # Initialize the test node and the controller interface
    test_node = Node("behavior_server_test")
    behavior_server_interface = BehaviorInterface(test_node)

    # Call the action server to spin the robot. Turn 90 degrees to the right.
    # TODO

    # Call the action server to wait. Wait for two seconds
    # TODO

    # Call the action server to spin the robot. Turn 90 degrees to the left.
    # TODO

    # Kill them all
    rclpy.shutdown()


if __name__ == "__main__":
    test_behavior_server()
