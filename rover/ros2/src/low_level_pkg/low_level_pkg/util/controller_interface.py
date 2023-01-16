# Import the ROS required modules
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import the required ROS interfaces
from nav_msgs.msg import Path
from nav2_msgs.action import # TODO
from geometry_msgs.msg import PoseStamped

# Import this to type the functions
from typing import List

# These imports are only used in the tests
# Import the get package function
from ament_index_python.packages import get_package_share_directory

# Import the yaml package
import yaml

class ControllerInterface:

    def __init__(self, parent_node: Node):
        """! Initialize the controller server interface.
        @param parent_node "Node" node instance that will be used to initialize
            the ROS-related attributes of class.
        """
        self.node = parent_node
        self.logger = parent_node.get_logger()
        self.controller_server_client = ActionClient(parent_node, # TODO )

    def call_action_client(self, frame_id: str, poses: List[PoseStamped], controller_id: str, goal_checker_id: str):
        """! Call the controller server action to follow a given path.
        @param frame_id "str" name of the frame id in which the points are.
        @param poses "List[PoseStamped]" list of PoseStamped elements that make
            up the path.
        @param controller_id "str" name of the controller that will be used.
        @param goal_checker_id "str" name of the goal checker that will be used.
        """
        # Check if controller server is not available
        if not self.controller_server_client.wait_for_server(timeout_sec=1.0):
            self.logger.error("Controller server is not available!")
            return

        action_goal = # TODO

        # Send the goal to the server
        future = self.controller_server_client.send_goal_async(action_goal)

        # Wait unitl the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the goal was accepted
        if not future.result().accepted:
            self.logger.error("The controller server goal was rejected by server!")
            return

        # Return the action result
        future = future.result().get_result_async()

        # Wait unitl the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Return the result
        return future.result()


def read_path() -> List[PoseStamped]:
    """! Function that generates a path to test the controller server.
    @return "str" name of the frame for the path.
    @return "List[PoseStamped]" list of PoseStamped elements with the path
        waypoints.
    @return "str" name of the used controller.
    @return "str" name of the used goal checker.
    """

    # TODO

    # Return the poses list
    return frame_id, poses, controller_id, goal_checker_id


def test_controller_server(args=None):
    """Function to test the controller server interface."""
    
    # Initialize rclpy
    rclpy.init(args=args)

    # Initialize the test node and the controller interface
    test_node = Node("controller_server_test")
    controller_server_interface = ControllerInterface(test_node)

    # Read the path
    frame_id, poses, controller_id, goal_checker_id = read_path()

    # Call the action client with the path
    result = controller_server_interface.call_action_client(frame_id, poses, controller_id, goal_checker_id)

    # Kill them all
    rclpy.shutdown()


if __name__ == "__main__":
    test_controller_server()
