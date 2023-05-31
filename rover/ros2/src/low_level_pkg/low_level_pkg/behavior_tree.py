# Import 'rclpy'
import rclpy

# Import the 'Node' class from rclpy
from rclpy.node import Node

# Import the Nav2's servers interfaces
from low_level_pkg.util.planner_interface import PlannerInterface
from low_level_pkg.util.behavior_interface import BehaviorInterface
from low_level_pkg.util.controller_interface import ControllerInterface

# Import the required ROS interfaces
from geometry_msgs.msg import PoseStamped
from nav2_msgs.msg import BehaviorTreeLog
from std_msgs.msg import String

import yaml


class BehaviorTree(Node):

    def __init__(self):

        # Initialize the super class
        super().__init__("low_level")

        # Create the instances of the interfaces
        self.planner_interface = PlannerInterface(self)
        self.behavior_interface = BehaviorInterface(self)
        self.controller_interface = ControllerInterface(self)

        # Create a publisher to play the song
        self.audio_pub = self.create_publisher(
            String, '/device/speaker/command', 10)

    def read_waypoints_f1(self):
        """! Function that generates a path to test the controller server.
        @return "str" name of the frame for the path.
        @return "List[PoseStamped]" list of PoseStamped elements with the path
            waypoints.
        @return "str" name of the used controller.
        @return "str" name of the used goal checker.
        @return "list" list of the sping angle to use
        @return "list" list of the track_name
        @return "list" list of the delay time
        """

        with open("/workspace/rover/ros2/src/tb_bringup/config/waypoints_mixed.yaml", 'r') as f:
            # data = f.read()
            data = yaml.safe_load(f)
            waypoints = data["waypoints"]

        frame_id = 'map'
        controller_id = 'diff_drive_controller'
        goal_checker_id = 'pose_error_checker'

        # Return the poses list
        # return frame_id, poses, controller_id, goal_checker_id, spin_angle, status, delay_time
        return frame_id, controller_id, goal_checker_id, waypoints

    def nav_order(self):
        """
        function that recognize the path order dynamically
        if you want to change the order just change the file named path_order.yaml
        the file is located in the same folder as the waypoints
        /workspace/rover/ros2/src/tb_bringup/config/path_order.yaml
        @return "list" the list order to follow
        """
        with open("/workspace/rover/ros2/src/tb_bringup/config/path_order.yaml", 'r') as f:
            # data = f.read()
            data = yaml.safe_load(f)
            waypoints = data["order"]
            path_order = waypoints

            nav_order = [list(path_order.items())[i][1]
                         for i in range(len(list(path_order.items())))]

        return nav_order

    def navigation(self):
        """ 
        Function that follow the waypoints, spin the robot and play the audio 
        when achieved the goal.
        """
        path_order = self.nav_order()
        # frame_id, poses, controller_id, goal_checker_id, spin_angle, status, delay_time = self.read_waypoints_f1()

        frame_id, controller_id, goal_checker_id, waypoints = self.read_waypoints_f1()

        for nav in path_order:
            poses = PoseStamped()
            poses.header.frame_id = frame_id
            poses.pose.position.x = waypoints[nav]["pose"]["x"]
            poses.pose.position.y = waypoints[nav]["pose"]["y"]

            spin_angle = waypoints[nav]["orientation"]["yaw"]
            delay_time = waypoints[nav]["wait_time"]

            # Navigate to the goal
            planner = self.planner_interface.call_action_client(
                frame_id, poses, controller_id, goal_checker_id)

            """
            Play the audio depend according to the goal.
            if in the yaml file the track_name is miss the robot
            must be follow the route to complete the order
            """
            try:
                track_name = waypoints[nav]["track"]
                msg = String()
                msg.data = track_name
                self.audio_pub.publish(msg)
            except:
                pass

            # The robot spin
            spin_robot = self.behavior_interface.call_spin_action_client(
                spin_angle)

            # Finally the robot wait
            self.behavior_interface.call_wait_action_client(int(delay_time))


def behavior_tree():

    rclpy.init()
    bt = BehaviorTree()
    delivery = bt.navigation()


if __name__ == "__main__":
    behavior_tree()
