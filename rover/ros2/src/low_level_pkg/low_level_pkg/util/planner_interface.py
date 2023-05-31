# Import the ROS required modules
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Import the required ROS interfaces
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from geometry_msgs.msg import PoseStamped

# Import this to type the functions
from typing import List

# These imports are only used in the tests
# Import the get package function
import yaml
from ament_index_python.packages import get_package_share_directory


class PlannerInterface:

    def __init__(self, parent_node: Node):
        """! Initialize the planner server interface.
        @param parent_node "Node" node instance that will be used to initialize
            the ROS-related attributes of class.
        """
        self.node = parent_node
        self.logger = parent_node.get_logger()
        self.planner_server_client = ActionClient(
            parent_node, NavigateToPose, 'navigate_to_pose')

    def call_action_client(self, frame_id: str, waypoints: List[PoseStamped], controller_id: str, goal_checker_id: str):
        """
        ! Call the planner server action to calculate a path.
        @param ...
        """
        # Check if controller server is not available
        if not self.planner_server_client.wait_for_server(timeout_sec=1.0):
            self.logger.error("Planner server is not available!")
            return

        # Set the goal to the server
        action_goal = NavigateToPose.Goal()
        action_goal.pose = waypoints
        action_goal.pose.header.frame_id = frame_id

        # Send the goal to the server
        future = self.planner_server_client.send_goal_async(action_goal)

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Check if the goal was accepted
        if not future.result().accepted:
            self.logger.error(
                "The planner server goal was rejected by server!")
            return

        # Return the action result
        future = future.result().get_result_async()

        # Wait until the future completes
        rclpy.spin_until_future_complete(self.node, future)

        # Return the action result
        return future.result()


def read_waypoints() -> List[PoseStamped]:
    """! Function that generates a path to test the controller server.
    @return "str" name of the frame for the path.
    @return "List[PoseStamped]" list of PoseStamped elements with the path
        waypoints.
    @return "str" name of the used controller.
    @return "str" name of the used goal checker.
    """

    # IF YOU WANT TO DO A CIRCLE TRAYECTORY CHANGE THE NAME "waypoints.yaml" TO "circle.yaml"
    with open("/workspace/rover/ros2/src/tb_bringup/config/waypoints.yaml", 'r') as f:
        data = f.read()

    poses = []
    lines = data.split("\n")

    for line in lines:
        if "x:" in line:
            x = float(line.split(":")[1])
        if "y:" in line:
            y = float(line.split(":")[1])

            point = PoseStamped()
            # point.header.frame_id = "map"
            point.pose.position.x = x
            point.pose.position.y = y
            poses.append(point)

    frame_id = "map"
    controller_id = 'diff_drive_controller'
    goal_checker_id = 'pose_error_checker'

    # Return the poses list
    return frame_id, poses, controller_id, goal_checker_id


def test_planner_server(args=None):

    # Initialize rclpy
    rclpy.init(args=args)

    # Initialize the test node and the controller interface
    test_node = Node("planner_server_test")
    planner_server_interface = PlannerInterface(test_node)

    # Generate the goal, start, planner ID and user start values
    frame_id, poses, controller_id, goal_checker_id = read_waypoints()

    for i in range(len(poses)):
        point = PoseStamped()
        # point.header.frame_id = "map"
        point.pose.position.x = poses[i].pose.position.x
        point.pose.position.y = poses[i].pose.position.y

        # Call the action client with the path
        result = planner_server_interface.call_action_client(
            frame_id, point, controller_id, goal_checker_id)

    # Kill them all
    rclpy.shutdown()


if __name__ == "__main__":
    test_planner_server()

"""
[1.0%] Which field of the planner server action should be set to use a specific planner?
The specific planner to be used is configured in the launch file or yaml file,
in the nav2_params in FollowPath.plugin located in planner server

[1.0%] What is the information that is provided in the planner server action feedback?
The feedback from the planner server action contains information about the progress 
of the current goal, such as the current pose of the robot, the distance remaining to t
he goal, and any errors or warnings that have occurred during planning.

[1.0%] Which costmap is used by the planner server?
The planner server uses a costmap to plan a path for the robot. The costmap is generated 
by the costmap_2d package and contains information about the environment, such as obstacles 
and the location of the robot.

[1.0%] Is sensor data being used by the planner server? If so, name a topic that is used
Yes, sensor data is used by the planner server. The planner server uses sensor data from 
the /scan topic to update the costmap and avoid obstacles.

[1.0%] Name three (3) planners that could be used by Nav2:

global_planner
local_planner
dwb_planner

[2.0%] Which is the error code for an invalid planner on a planner server action call?
ActionToPoseGoal::INVALID_PLANNER

[1.0%] What would happen if you plan a path toward a goal that is located outside of the costmap?
The node return an error saying [bt_navigator]: Goal failed

[1.0%] What would happen if you plan a path toward a goal that is within the costmap but unreachable by the robot?
The robot try to achieve the goal but after a couple of seconds return a error saying [bt_navigator]: Goal failed

[1.0%] What would happen if you plan a path toward a goal that is the same as the start pose:
return a message said goal achieved

"""
