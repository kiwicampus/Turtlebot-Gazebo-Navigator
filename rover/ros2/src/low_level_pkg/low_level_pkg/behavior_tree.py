# Import 'rclpy'
import rclpy

# Import the 'Node' class from rclpy 
from rclpy.node import Node

# Import the Nav2's servers interfaces
from low_level_pkg.util.planner_interface import PlannerInterface
from low_level_pkg.util.behavior_interface import BehaviorInterface
from low_level_pkg.util.controller_interface import ControllerInterface


class BehaviorTree(Node):

    def __init__(self):

        # Initialize the super class
        super().__init__("low_level")

        # Create the instances of the interfaces
        self.planner_interface = PlannerInterface(self)
        self.behavior_interface = BehaviorInterface(self)
        self.controller_interface = ControllerInterface(self)

    # TODO

def behavior_tree():

    rclpy.init()

    bt = BehaviorTree()

if __name__ == "__main__":
    behavior_tree()