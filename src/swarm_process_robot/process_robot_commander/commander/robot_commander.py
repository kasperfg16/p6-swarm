#! /usr/bin/env python3
# reference: https://automaticaddison.com/how-to-send-goals-to-the-ros-2-navigation-stack-nav2/

from tabnanny import check
from time import sleep, time
from tkinter import PROJECTING
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult
import rclpy
import transformations as tf
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.action import ActionClient
from docking_action_server.action import Docking
from process_robot_commander.action import Feeder
from std_msgs.msg import String
from std_msgs.msg import Int32

class feedPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int32, 'brickfeeder_goal', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.num_bricks = 0

    def timer_callback(self):
        msg = Int32()
        msg.data = self.num_bricks
        self.publisher_.publish(msg)
        if self.i <= 10:
            self.get_logger().info('Requesting feeder for feeding of "%s" bricks' % msg.data)
            self.i = 0
        self.i += 1

class feedSubscriber(Node):

    def __init__(self):
        super().__init__('feeder_subscriber')
        self.subscription = self.create_subscription(
            String,
            'brickfeeder_status',
            self.feed_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.feeding = False

    def feed_callback(self, msg):
        if msg.data == "G1":
            self.get_logger().info('Feeder booting')
            self.feeding = False

        elif msg.data == "G2":
            self.get_logger().info('Feeder ready for new goal')
            self.feeding = False
            self.fed = True

        elif msg.data == "G3":
            self.feeding = True
            self.fed = False


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
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Docked: {0}'.format(result.docked))
        self.docked = result.docked


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Is docking: {0}'.format(feedback.is_docking))
        self.is_docking = feedback.is_docking
        

def run_forrest_run(x, y, z, roll, pitch, yaw, navigator, reference='map'):

    # Set the robot's goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = reference
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

def send_goal(x, y, z, roll, pitch, yaw, num_bricks, docking_action_client, feeder_subscriber, feeder_publisher, navigator, refrenceframe):
    # Set the robot's init pose
    init_pose = PoseStamped()
    init_pose.header.frame_id = 'base_footprint'
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

    result = run_forrest_run(x, y, z, roll, pitch, yaw, navigator, refrenceframe)    

    if result == NavigationResult.SUCCEEDED:

        # Request docking
        print("Requesting docking")
        docking_action_client.request_dock()
        while not docking_action_client.docked:
            rclpy.spin_once(docking_action_client)

        if docking_action_client.docked:
            # Request Feeder starting
            i = 0
            feeder_publisher.num_bricks = num_bricks
            while not feeder_subscriber.feeding:
                rclpy.spin_once(feeder_publisher)
                rclpy.spin_once(feeder_subscriber)
            
            feeder_subscriber.get_logger().info('Feeder is feeding')

            while not feeder_subscriber.fed:
                rclpy.spin_once(feeder_subscriber)

            feeder_subscriber.get_logger().info('Feeder has fed')
            
            print('hurray! I docked and fed some hot carrier bot')
    
    elif result == NavigationResult.CANCELED:
        print('Goal was canceled!')
    elif result == NavigationResult.FAILED:
        print('Goal failed!')

def main(args=None):
    
    # Start the ROS 2 Python Client Library
    rclpy.init(args=args)

    # Create instance of docking action client
    docking_action_client = DockingActionClient()

    # Create instance of feeder publisher
    feeder_publisher = feedPublisher()

    # Create instance of feeder subscriber
    feeder_subscriber = feedSubscriber()

    # Launch the ROS 2 Navigation Stack
    navigator = BasicNavigator()

    # Create a goal for process robot
    x = 1.0
    y = 0.0
    z = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    num_bricks = 3
    refrenceframe = "odom"

    send_goal(
    x, 
    y, 
    z, 
    roll, 
    pitch,  
    yaw, 
    num_bricks,
    docking_action_client,
    feeder_subscriber,
    feeder_publisher,
    navigator,
    refrenceframe=refrenceframe)

if __name__ == '__main__':
    main()
