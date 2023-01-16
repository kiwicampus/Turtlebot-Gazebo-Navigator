# Import the ROS required modules
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import the required ROS interfaces
from nav2_msgs.action import # TODO
from geometry_msgs.msg import PoseStamped

# These imports are only used in the tests
# Import the get package function
import yaml
from ament_index_python.packages import get_package_share_directory


class PlannerInterface:

    def __init__(self, parent_node: Node):
        """! Initialize the planner server interface.
        @param parent_node "Node" node instance that will be used to initialize
            the ROS-related attributes of class.
        """
        self.node = parent_node
        self.logger = parent_node.get_logger()
        self.planner_server_client = ActionClient(parent_node, # TODO )

    def call_action_client(self,  # TODO ):
        """! Call the planner server action to calculate a path.

        @param ...

        """
        # Check if controller server is not available
        if not self.planner_server_client.wait_for_server(timeout_sec=1.0):
            self.logger.error("Planner server is not available!")
            return

        action_goal = # TODO

        # Send the goal to the server
        future = self.planner_server_client.send_goal_async(action_goal)

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the goal was accepted
        if not future.result().accepted:
            self.logger.error("The planner server goal was rejected by server!")
            return

        # Return the action result
        future = future.result().get_result_async()

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Return the action result
        return future.result()


def # TODO:


def test_planner_server(args=None):
    
    # Initialize rclpy
    rclpy.init(args=args)

    # Initialize the test node and the controller interface
    test_node = Node("planner_server_test")
    planner_server_interface = PlannerInterface(test_node)

    # Generate the goal, start, planner ID and user start values
    # TODO

    # Call the action client with the path
    result = planner_server_interface.call_action_client(# TODO)

    # Kill them all
    rclpy.shutdown()

if __name__ == "__main__":
    test_planner_server()