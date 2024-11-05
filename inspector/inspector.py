import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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
        self._cbgroup = MutuallyExclusiveCallbackGroup()
        self._server = ActionServer(self,
                                    MoveGroup,
                                    '/viz/move_action',
                                    self.move_action_callback,
                                    callback_group = self._cbgroup)
        self._client = ActionClient(self, MoveGroup, 'move_action')
        if not self._client.wait_for_server(timeout_sec=10):
            raise RuntimeException("move_action action server not ready")


    async def move_action_callback(self, goal_handle):
        self.get_logger().info("Move Action Called. Goal Is")
        self.get_logger().info(f"{goal_handle.request}") # Do more to format this output nicelynicely
        self.get_logger().info(f"--- End ofrequest dump.----\n")
        self.get_logger().info("Forwarding the action to the move_group action server")
        response_goal_handle = await self._client.send_goal_async(goal_handle.request)
        self.get_logger().info(f"Received response goal handle: Accepted? {response_goal_handle.accepted}")
        self.get_logger().info("Awaiting the result")
        response = await response_goal_handle.get_result_async()
        self.get_logger().info(f"Received the result: {response}") # Do more formatting
        goal_handle.succeed()
        self.get_logger().info("Interception successful, returning the result")
        return response.result

def main(args=None):
    rclpy.init()
    node = Inspector()
    rclpy.spin(node)
