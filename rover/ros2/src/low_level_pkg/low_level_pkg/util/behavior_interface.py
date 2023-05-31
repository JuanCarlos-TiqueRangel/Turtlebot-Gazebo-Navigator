# Import the ROS required modules
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import the required ROS interfaces
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose, Spin, Wait, BackUp

# Import the builtin 'Duration' message
from builtin_interfaces.msg import Duration

import math

# Import this to type the functions
from typing import List


class BehaviorInterface:

    def __init__(self, parent_node: Node):
        """! Initialize the controller server interface.
        @param parent_node "Node" node instance that will be used to initialize
            the ROS-related attributes of class.
        """
        self.node = parent_node
        self.logger = parent_node.get_logger()
        self.behavior_spin_server_client = ActionClient(
            parent_node, Spin, 'spin')
        self.behavior_wait_server_client = ActionClient(
            parent_node, Wait, 'wait')
        self.behavior_backup_server_client = ActionClient(
            parent_node, BackUp, 'backup')

    def call_spin_action_client(self, spin_angle: float):
        """
        ! Call the spin server action to calculate a path.
        @param ...
        """
        # Check if spin server is not available
        if not self.behavior_spin_server_client.wait_for_server(timeout_sec=1.0):
            self.logger.error("Behavior server is not available!")
            return

        action_goal = Spin.Goal()
        action_goal.target_yaw = spin_angle
        # action_goal.time_allowance.sec.

        # Send the goal to the server
        future = self.behavior_spin_server_client.send_goal_async(action_goal)

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the goal was accepted
        if not future.result().accepted:
            self.logger.error(
                "The Behavior server goal was rejected by server!")
            return

        # Return the action result
        future = future.result().get_result_async()

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Return the action result
        return future.result()

    def call_wait_action_client(self, wait_time: int):
        """
        ! Call the wait server action to calculate a path.
        @param ...
        """
        # Check if wait server is not available
        if not self.behavior_wait_server_client.wait_for_server(timeout_sec=1.0):
            self.logger.error("Behavior server is not available!")
            return

        action_goal = Wait.Goal()
        action_goal.time.sec = wait_time
        # action_goal.time_allowance.sec.

        # Send the goal to the server
        future = self.behavior_wait_server_client.send_goal_async(action_goal)

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the goal was accepted
        if not future.result().accepted:
            self.logger.error(
                "The Behavior server goal was rejected by server!")
            return

        # Return the action result
        future = future.result().get_result_async()

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Return the action result
        return future.result()

    def call_backup_action_client(self, target_x: float, target_y: float, speed: float):
        """
        ! Call the backup server action to calculate a path.
        @param ...
        """
        # Check if wait server is not available
        if not self.behavior_backup_server_client.wait_for_server(timeout_sec=1.0):
            self.logger.error("Behavior server is not available!")
            return

        action_goal = BackUp.Goal()
        action_goal.target.x = target_x
        action_goal.target.y = target_y
        action_goal.speed = speed

        # Send the goal to the server
        future = self.behavior_backup_server_client.send_goal_async(
            action_goal)

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the goal was accepted
        if not future.result().accepted:
            self.logger.error(
                "The Behavior server goal was rejected by server!")
            return

        # Return the action result
        future = future.result().get_result_async()

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Return the action result
        return future.result()


def test_behavior_server(args=None):

    # Initialize rclpy
    rclpy.init(args=args)

    # Initialize the test node and the controller interface
    test_node = Node("behavior_server_test")
    behavior_server_interface = BehaviorInterface(test_node)

    # Call the action server to spin the robot. Turn 90 degrees to the right.
    result = behavior_server_interface.call_spin_action_client(-1.57)

    # Call the action server to wait. Wait for two seconds
    result = behavior_server_interface.call_wait_action_client(2)

    # Call the action server to spin the robot. Turn 90 degrees to the left.
    result = behavior_server_interface.call_spin_action_client(1.57)

    # Call the action server to run the backup on the robot.
    result = behavior_server_interface.call_backup_action_client(
        1.0, 0.0, 0.2)

    # Kill them all
    rclpy.shutdown()


if __name__ == "__main__":
    test_behavior_server()


""" 
[1.0%] What is the main reason the behavior server exists in Nav2?
The main reason the behavior server exists in Nav2 is to provide a central 
point for coordinating the robot's behaviors. This allows for different components 
of the system to interact with the robot's behaviors in a consistent and predictable way.

[1.0%] Is playing an alert sound valid as a behavior within the context of the behavior server?
It is possible to play an alert sound as a behavior within the context of the behavior server. It 
would be a custom behavior, and it would be up to the implementation to decide how it would be 
activated and what sound to play

[1.0%] What is the purpose of the assisted teleop behavior?
The purpose of the assisted teleop behavior is to allow a user to manually control the robot's movement 
while still providing some level of autonomy. This can be useful in situations where the robot's automatic 
navigation is not able to handle the task at hand.

[1.0%] Which situation might trigger a Clear Costmap behavior? ...
A situation that might trigger a Clear Costmap behavior is if the robot's sensors detect an obstacle or a 
dynamic object that is not being properly accounted for by the costmap. This behavior would clear the costmap 
of the obstacle, so that the robot can continue its navigation

[1.0%] Within the context of Nav2, what does it mean that each behavior has its own API?
Within the context of Nav2, the fact that each behavior has its own API means that each behavior is implemented 
and controlled independently. This allows for behaviors to be added, modified, or removed from the system without 
affecting the other behaviors. It also allows for different behaviors to have different parameters and inputs, and 
for the behavior server to send commands and receive feedback for each behavior separately.
"""
