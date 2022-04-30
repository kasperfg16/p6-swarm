#! /usr/bin/env python3
# reference: https://automaticaddison.com/how-to-send-goals-to-the-ros-2-navigation-stack-nav2/

from tabnanny import check
from tkinter import PROJECTING
from unittest import case
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
import rclpy
import transformations as tf
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from docking_action_server.action import Docking
from process_robot_commander.action import Feeder

class FeederActionClient(Node):

    def __init__(self):
        super().__init__('feeder_action_client')
        self._action_client = ActionClient(self, Feeder, 'feeder')
        self.feeder_done = False
        self.is_feeding = False
        self.bricks_fed_so_far = 0

    def request_feeding(self, num_bricks):
        goal_msg = Feeder.Goal()
        goal_msg.start_feeder = True
        goal_msg.num_bricks = num_bricks

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Feeder done: {0}'.format(result.feeder_done))
        self.get_logger().info('Feeder done: {0}'.format(result.num_bricks_fed))
        self.feeder_done = result.feeder_done

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Is feeding: {0}'.format(feedback.is_feeding))
        self.get_logger().info('Bricks fed so far: {0}'.format(feedback.bricks_fed_so_far))
        self.is_feeding = feedback.is_feeding
        self.bricks_fed_so_far = feedback.bricks_fed_so_far



class DockingActionClient(Node):

    def __init__(self):
        super().__init__('docking_action_client')
        self._action_client = ActionClient(self, Docking, 'docking')
        self.docked = False
        self.is_docking = False

    def request_dock(self):
        goal_msg = Docking.Goal()
        goal_msg.start_docking = True
        goal_msg.carrier_id = 3
        goal_msg.feeder_id = 1

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        print(self._get_result_future)
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Docked: {0}'.format(result.docked))
        rclpy.shutdown()
        self.docked = result.docked
        print(result.docked)


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Is docking: {0}'.format(feedback.is_docking))
        self.is_docking = feedback.is_docking
        

def run_forrest_run(x, y, z, roll, pitch, yaw, navigator):

    # Set the robot's goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    # Position
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z

    # Orientation
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    goal_pose.pose.orientation.x = quaternion[0]
    goal_pose.pose.orientation.y = quaternion[1]
    goal_pose.pose.orientation.z = quaternion[2]
    goal_pose.pose.orientation.w = quaternion[3]

    # Go to the goal pose
    navigator.goToPose(goal_pose)
    i = 0
    while not navigator.isNavComplete():
        i += 1
        feedback = navigator.getFeedback()
        if i == 5:
            #print('Distance remaining: ' + '{:.2f}'.format(
            #    feedback.distance_remaining) + ' meters.')
            i = 0

    result = navigator.getResult()
    if result == NavigationResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == NavigationResult.CANCELED:
        print('Goal was canceled!')
    elif result == NavigationResult.FAILED:
        print('Goal failed!')

    return result
    
def main(args=None):
    # Start the ROS 2 Python Client Library
    rclpy.init(args=args)

    # Create instance of docking action client
    docking_action_client = DockingActionClient()

    # Create instance of feeder action client
    feeder_action_client = FeederActionClient()

    # Launch the ROS 2 Navigation Stack
    navigator = BasicNavigator()

    # Set the robot's init pose
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = navigator.get_clock().now().to_msg()

    # Position
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    init_pose.pose.position.z = 0.0

    # Orientation
    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    init_pose.pose.orientation.x = quaternion[0]
    init_pose.pose.orientation.y = quaternion[1]
    init_pose.pose.orientation.z = quaternion[2]
    init_pose.pose.orientation.w = quaternion[3]

    navigator.setInitialPose(init_pose)

    x = 2.9
    y = 2.9
    z = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 3.14/2
    result = run_forrest_run(x, y, z, roll, pitch, yaw, navigator)

    if result == NavigationResult.SUCCEEDED:

        # Request docking
        print("Request docking")
        docking_action_client.request_dock()
        rclpy.spin(docking_action_client)
        docked = docking_action_client.docked
        if docked:
            # Request Feeder starting
            print("Request feeder")
            feeder_action_client.request_feeding(num_bricks=2)
            has_fed = feeder_action_client.get_result_callback()
            if has_fed:
                print('hurray! I docked and fed some hot carrier bot')
    
    elif result == NavigationResult.CANCELED:
        print('Goal was canceled!')
    elif result == NavigationResult.FAILED:
        print('Goal failed!')

if __name__ == '__main__':
    main()
