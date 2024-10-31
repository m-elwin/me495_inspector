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

    def move_action_callback(self, goal):
        self.get_logger().info("Move Action Called. Goal Is")
        self.get_logger().info(goal)
        self.get_logger().info("END GOAL")
        goal.succeed()
        return MoveGroup.result

def main(args=None):
    rclpy.init()
    node = Inspector()
    rclpy.spin(node)
