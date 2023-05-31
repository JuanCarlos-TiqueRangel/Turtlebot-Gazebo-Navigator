# Import the ROS required modules
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import the required ROS interfaces
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath, NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped

# Import this to type the functions
from typing import List

# These imports are only used in the tests
# Import the get package function
from ament_index_python.packages import get_package_share_directory

# Import the yaml package
import yaml
import os


class ControllerInterface:

    def __init__(self, parent_node: Node):
        """! Initialize the controller server interface.
        @param parent_node "Node" node instance that will be used to initialize
            the ROS-related attributes of class.
        """
        self.node = parent_node
        self.logger = parent_node.get_logger()
        self.controller_server_client = ActionClient(
            parent_node, NavigateToPose, 'navigate_to_pose')

        """ 
        ActionClient parms (node, action_type, action_name)
        action_name:
            - navigate_to_pose
            - navigate_through_poses
        """

    def call_action_client(self, frame_id: str, poses: List[PoseStamped], controller_id: str, goal_checker_id: str):
        """! Call the controller server action to follow a given path.
        @param frame_id "str" name of the frame id in which the points are.
        @param poses "List[PoseStamped]" list of PoseStamped elements that make up the path.
        @param controller_id "str" name of the controller that will be used.
        @param goal_checker_id "str" name of the goal checker that will be used.
        """

        # Check if controller server is not available
        if not self.controller_server_client.wait_for_server(timeout_sec=1.0):
            self.logger.error("Controller server is not available!")
            return

        action_goal = NavigateToPose.Goal()
        action_goal.pose = poses

        # Send the goal to the server
        future = self.controller_server_client.send_goal_async(action_goal)

        # Wait unitl the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the goal was accepted
        if not future.result().accepted:
            self.logger.error(
                "The controller server goal was rejected by server!")
            return

        # Return the action result
        future = future.result().get_result_async()

        # Wait unitl the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Return the result
        return future.result()


def read_path() -> List[PoseStamped]:
    """! Function that generates a path to test the controller server.
    @return "str" name of the frame for the path.
    @return "List[PoseStamped]" list of PoseStamped elements with the path
        waypoints.
    @return "str" name of the used controller.
    @return "str" name of the used goal checker.
    """

    # Read the yaml file as a simple text file
    with open("/workspace/rover/ros2/src/tb_bringup/config/circle.yaml", 'r') as f:
        data = f.read()

    poses = []
    lines = data.split("\n")

    # split the positions to get each of them
    for line in lines:
        if "x:" in line:
            x = float(line.split(":")[1])
        if "y:" in line:
            y = float(line.split(":")[1])

            point = PoseStamped()
            point.header.frame_id = "map"
            point.pose.position.x = x
            point.pose.position.y = y
            poses.append(point)

    frame_id = 'map'
    controller_id = 'diff_drive_controller'
    goal_checker_id = 'pose_error_checker'

    # Return the poses list
    return frame_id, poses, controller_id, goal_checker_id


def test_controller_server(args=None):
    """Function to test the controller server interface."""

    # Initialize rclpy
    rclpy.init(args=args)

    # Initialize the test node and the controller interface
    test_node = Node("controller_server_test")
    controller_server_interface = ControllerInterface(test_node)

    # Read the path
    frame_id, poses, controller_id, goal_checker_id = read_path()

    for i in range(len(poses)):
        print("pos: %f valor i: %i", poses[i].pose.position.x, i)

        point = PoseStamped()
        point.header.frame_id = "map"
        point.pose.position.x = poses[i].pose.position.x
        point.pose.position.y = poses[i].pose.position.y

        # Call the action client with the path
        result = controller_server_interface.call_action_client(
            frame_id, point, controller_id, goal_checker_id)

        # print(i)

    # Kill them all
    rclpy.shutdown()


if __name__ == "__main__":
    test_controller_server()

""" 
SECTION 1: CONCEPTUAL QUESTIONS
### [0.5%] What is the difference between an action, a topic and a service in ROS? ###
In the Robot Operating System (ROS), an action is a type of message exchanged between
nodes that allows for more complex communication patterns than simple request-response
or publish-subscribe. An action message includes a goal, feedback, and a result, and 
allows for things like canceling or pausing goals.

A topic is a named bus on which nodes can send and receive messages. Topics allow for 
one-to-many communication, where multiple nodes can subscribe to receive messages, and 
multiple nodes can also publish messages to a topic.

A service is a way for nodes to send a request and receive a response, like a function 
call. Services are typically used for simple request-response communication, and allow 
for one-to-one communication, where a single node sends a request and another node responds.

### [0.5%]  What is the use of the controller server within the context of Nav2? #######
In the context of Nav2, the controller server is a component that receives navigation goals 
and generates control commands for a robot to execute in order to reach those goals. T
he controller server is responsible for path planning and trajectory generation and uses a 
costmap to avoid obstacles.

### [0.5%]  What is the name and type of the controller server action?
The name of the controller server action is "navigate" and its type is nav2_msgs/action/Navigate.

### [1.0%] What are the fields of the controller server action and what is their purpose?
The fields of the controller server action are goal, feedback and result. The goal field contains 
the navigation goal, the feedback field contains information about the progress of the navigation, 
and the result field contains the final result of the navigation.

### [0.5%]  Which costmap is used by the controller server?
The controller server uses the global costmap.

### [1.0%] Describe the controller, progress checker, and goal checker in the context of Nav2's controller server
The controller is the component that takes the navigation goal and generates control commands for the robot 
to execute. The progress checker is a component that checks the progress of the robot towards the goal and 
updates the feedback field of the action message. The goal checker is a component that checks if the goal 
has been reached and updates the result field of the action message

### [1.0%] What is a future in the context of ROS asynchronous calls?
In the context of ROS asynchronous calls, a future is an object that represents the outcome of an asynchronous 
operation. It allows the caller to check the status of the operation and retrieve the result, but doesn't block 
the execution while waiting for the result. It allows to continue the execution of other tasks while waiting 
for the asynchronous call to complete

### [1.0%] Name two or three additional examples for controller servers.
- FollowJointTrajectoryController
- JointPositionController
- JointVelocityController

### [0.5%] Which is the default controller that is used in Nav2's controller server?
The default controller that is used in Nav2's controller server is the dwb_controller

### [0.5%] Name three of the available controller plugins.
- dwb_controller: a controller that implements the Dynamic Window Approach (DWA) algorithm for path 
planning and trajectory generation.

- teb_local_planner: a controller that implements the Time Elastic Band (TEB) algorithm 
for path planning and trajectory generation.

- simple_controller: a simple controller that sends velocity commands to the robot based 
on the desired goal

### [1.0%] What should I modify if my speed control topic has the name /kiwi/output_cmd ?
If your speed control topic has the name /kiwi/output_cmd, you should modify the velocity_command_topic 
parameter in the controller's configuration file to match that name.

### [1.0%] What is the purpose of using the spin_until_future_complete function in the call_action_client?
The purpose of using the spin_until_future_complete function in the call_action_client is to block 
the execution of the script until the action client receives a response from the action server.

### [1.0%] Why are two (2) future instances used in the call_action_client function?
Two future instances are used in the call_action_client function because one is used to 
track the status of the action call and the other is used to retrieve the result of the action 
call. This allows the script to check the status of the action call and retrieve the result in 
a non-blocking way.
"""
