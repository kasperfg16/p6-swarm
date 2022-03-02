#! /usr/bin/env python3
# https://automaticaddison.com/how-to-send-goals-to-the-ros-2-navigation-stack-nav2/

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
import transformations as tf


def main():
    # Start the ROS 2 Python Client Library
    rclpy.init()

    # Launch the ROS 2 Navigation Stack
    navigator = BasicNavigator()

    # Wait for navigation to fully activate. Use this line if autostart is set to true.
    navigator.waitUntilNav2Active()

    i = 0

    while True:

        x = i+1
        y = i+1
        z = 0
        roll = 0
        pitch = 0
        yaw = 3.14/2

        run_forrest_run(x, y, z, roll, pitch, yaw, navigator)

        if i == 0:
            i = 1
        else:
            i = 0


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
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback.navigation_duration > 600:
            navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')


if __name__ == '__main__':
    main()
