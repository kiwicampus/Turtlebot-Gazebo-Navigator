import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


class Nav2CommanderNode(Node):
    def __init__(self, node_name):
        Node.__init__(self, node_name=node_name)
        self.get_logger().info("Hi! I'm the nav2 commander node!")


def main():
    rclpy.init()
    nav2_commander = Nav2CommanderNode("nav2_commander_py")

    executor = MultiThreadedExecutor()
    executor.add_node(nav2_commander)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
