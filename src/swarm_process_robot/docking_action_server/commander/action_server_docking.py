import time


import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from docking_action_server.action import Docking

class DockingActionServer(Node):

    def __init__(self):
        super().__init__('docking_action_server')
        self._action_server = ActionServer(
            self,
            Docking,
            'docking',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Docking.Feedback()

        feedback_msg.is_docking = False
        
        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = Docking.Result()
        self.get_logger().info('Goal succeded...')

        result.docked = True
        return result

def main(args=None):
    rclpy.init(args=args)

    docking_action_server = DockingActionServer()

    rclpy.spin(docking_action_server)

if __name__ == '__main__':
    main()
    