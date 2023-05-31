#include "high_level_nav2_commander_cpp/high_level_commander.hpp"

Nav2CommanderNode::Nav2CommanderNode(const std::string& node_name) : Node(node_name, rclcpp::NodeOptions())
{
  // Create the action client
  this-> client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  // Create publisher
  pub_audio_ = this->create_publisher<std_msgs::msg::String>("/device/speaker/command", 10);

  //Run the test interface
  Nav2CommanderNode::test_interface_server();

}

YAML::Node read_path(){
  /*! Function that generates a path to test the controller server.
  @return "YAML::Node" list of PoseStamped elements with the path.*/

  //Read the waypoints.yaml file
  YAML::Node config = YAML::LoadFile("/workspace/rover/ros2/src/tb_bringup/config/waypoints.yaml");

  // Return a vector with the paths
  return config;
}

void Nav2CommanderNode::send_goal(const geometry_msgs::msg::PoseStamped &goal)
{
  // wait until action is provided
  while (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_INFO(get_logger(), "Waiting for action server...");
  }
  
  // SET THE GOAL TO THE SERVER
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose = goal;
  goal_msg.pose.header.frame_id = "map";

  //Set a Feedback callback to show progress
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.feedback_callback = std::bind(&Nav2CommanderNode::feedbackCallback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&Nav2CommanderNode::resultCallback, this, _1);

  //Send Goal to Server
  auto future = client_ptr_->async_send_goal(goal_msg, send_goal_options);

  if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(1)) != 
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "send_goal failed");
    return;
  }

  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(
      get_logger(), "ExecutorClient: Execution was rejected by the action server");
    return;
  }
}

//feedback
void Nav2CommanderNode::feedbackCallback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  /*
  print method to see the stimate time remaining and distance 
  estimated_time_remaining: type double  
  distance_remaining.sec: type double 
  */
  RCLCPP_INFO(get_logger(), "Time remaininf = %i s", feedback->estimated_time_remaining.sec);
}

//result
void Nav2CommanderNode::resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Success!!!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      return;
  }
}

void Nav2CommanderNode::test_interface_server()
{
  auto poses = geometry_msgs::msg::PoseStamped();
  auto track_name = std_msgs::msg::String();

  YAML::Node path = read_path();
  //std::cout << path["waypoints"]["restaurant"]["pose"]["x"].as<double>() << std::endl;

  for (YAML::iterator it = path["waypoints"].begin(); it != path["waypoints"].end(); ++it)
  {    
    poses.header.frame_id = "map";
    poses.pose.position.x = it->second["pose"]["x"].as<double>();
    poses.pose.position.y = it->second["pose"]["y"].as<double>();

    Nav2CommanderNode::send_goal(poses);

    // Publish the track_name and play the audio
    track_name.data = it->second["track"].as<std::string>();
    pub_audio_->publish(track_name);
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto nav2_commander = std::make_shared<Nav2CommanderNode>("nav2_commander_cpp");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nav2_commander);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}


/*
[1%] Can the two callbacks run simultaneously if the node is spun by a SingleThreadedExecutor?
NO, because It executes callbacks in a single thread, one at a time, and thus the previous callback 
must always finish before a new one can begin execution.

[1%] Can the two callbacks run simultaneously if the node is spun by a SingleThreadedExecutor 
but they belong to the same MutuallyExclusiveCallbackGroup?

No, the two callbacks cannot run simultaneously if the node is spun by a SingleThreadedExecutor and 
they belong to the same MutuallyExclusiveCallbackGroup. The callbacks in the same MutuallyExclusiveCallbackGroup 
are executed one at a time, in the order in which they were added to the group, and only one callback in the group c
an be executing at any time

[1%] Can the two callbacks run simultaneously if the node is spun by a SingleThreadedExecutor 
but they belong to different MutuallyExclusiveCallbackGroups?

The two callbacks cannot run simultaneously if the node is spun by a SingleThreadedExecutor 
and they belong to different MutuallyExclusiveCallbackGroups. In this case, the SingleThreadedExecutor 
ensures that callbacks are executed in a serialized manner, and the MutuallyExclusiveCallbackGroups further 
restrict the execution of certain callbacks so that they do not run concurrently with each other

[1%] Can the two callbacks run simultaneously if the node is spun by a SingleThreadedExecutor 
but they belong to the same ReentrantCallbackGroup?

NO, The SingleThreadedExecutor only allows one callback to run at a time, regardless of the callback group it belongs to.

[1%] Can the two callbacks run simultaneously if the node is spun by a MultiThreadedExecutor?
Yes, if the node is spun by a MultiThreadedExecutor, the two callbacks can run simultaneously. This is because a 
MultiThreadedExecutor uses multiple threads, allowing multiple callbacks to run in parallel

[1%] Can the two callbacks run simultaneously if the node is spun by a MultiThreadedExecutor 
but they belong to the same MutuallyExclusiveCallbackGroup?

No, callbacks that belong to the same MutuallyExclusiveCallbackGroup cannot run simultaneously, even 
if the node is spun by a MultiThreadedExecutor. MutuallyExclusiveCallbackGroups ensure that only one of 
the callbacks within the group can be executing at any given time.

[1%] Can the two callbacks run simultaneously if the node is spun by a MultiThreadedExecutor 
but they belong to different MutuallyExclusiveCallbackGroups?

Callbacks belonging to different MutuallyExclusiveCallbackGroups can run simultaneously in a 
MultiThreadedExecutor because each group is assigned a separate thread, allowing multiple groups to 
execute concurrently. This can lead to improved performance, but also requires careful consideration of 
thread-safety and synchronization issues.

[1%] Can the two callbacks run simultaneously if the node is spun by a MultiThreadedExecutor 
but they belong to the same ReentrantCallbackGroup?

Yes, the two callbacks can run simultaneously if the node is spun by a MultiThreadedExecutor and they belong to the same 
ReentrantCallbackGroup, as the callbacks in this group are designed to be executed concurrently by multiple threads.

[3%]What's the purpose of the nav2_params.yaml file?
The nav2_params.yaml file is a configuration file used to set parameters for the nav2 navigation stack. 
These parameters can control aspects such as planner selection, costmap configuration, and global and local 
planner tuning parameters.

[5% - EXTRA]What's the task of amcl? Why does't the BT interact with it while it does with most other servers?
amcl (Adaptive Monte Carlo Localization) is a probabilistic localization system for a robot moving in 2D. It is 
used to estimate the pose of the robot in a known map. BT (behavior tree) interacts with other servers to execute 
tasks and make decisions for the robot. However, it does not interact with amcl because amcl is a passive system that 
provides information to the robot, but does not execute tasks or make decisions.

[5%]What reference frames in REP105 are needed for nav2 to work? which of those is attached to the robot
In REP105, the required reference frames for nav2 are "map", "odom", and "base_link". The "map" frame is the static 
map of the environment. The "odom" frame is the odometry frame of the robot and is typically used for localization. 
The "base_link" frame is attached to the robot and represents the robot's body

[5% - EXTRA]Why do you think we have map and odom instead of a single frame?
The reason we have map and odom instead of a single frame is that the map frame is a fixed frame that represents 
the static environment, while the odom frame is a moving frame that represents the odometry of the robot. This 
distinction allows the navigation system to track the robot's movement relative to the environment

[5%]Why do you think nav2 needs two costmaps instead of just one?
Nav2 uses two costmaps instead of just one because one costmap is used for global planning, and the other is used 
for local planning. The global costmap is used for large-scale path planning and considers the entire environment. T
he local costmap is used for fine-tuning the robot's path in real-time and considers only the immediate surroundings of the robot

[5% - EXTRA]What's a costmap layer? what do we mean when we say nav2 costmaps are layered? Which kind of costmap 
layer would you use to populate the costmap with data coming from a 2D lidar?
A costmap layer is a specific piece of information that is added to the costmap to represent a particular aspect of 
the environment. Nav2 costmaps are layered because they combine multiple pieces of information, such as occupancy grid 
data from a laser scan, to create a complete representation of the environment. To populate the costmap with data from a 
2D lidar, you would use a "LaserScan" layer.

*/