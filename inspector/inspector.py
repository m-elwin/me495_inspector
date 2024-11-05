import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from moveit_msgs.action import MoveGroup

class Inspector(Node):
    """
    Action Servers
    --------------
    /viz/move_action : (moveit_msgs/action/MoveGroup) - intercepts move_actions from rviz

    Action Clients
    --------------
    move_action : (moveit_msgs/action/MoveGroup) - forwards the action as a request
    """

    def __init__(self):
        super().__init__("inspector")
        self._server = ActionServer(self,
                                    MoveGroup,
                                    '/viz/move_action',
                                    self.move_action_callback)

    def move_action_callback(self, goal_handle):
        self.get_logger().info("Move Action Called. Goal Is")
        self.get_logger().error(f"{goal_handle.request}")
        self.get_logger().info("END GOAL")
        goal_handle.succeed()
        return MoveGroup.Result()

def main(args=None):
    rclpy.init()
    node = Inspector()
    rclpy.spin(node)
