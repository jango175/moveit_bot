#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#define PACKAGE_NAME       "moveit_controller"
#define MOVE_GROUP_NAME    "panda_arm"


void get_current_pose(moveit::planning_interface::MoveGroupInterface *move_group)
{
  // Get the current pose of the robot's end-effector
  geometry_msgs::msg::PoseStamped current_pose = move_group->getCurrentPose();

  // Print the pose information
  RCLCPP_INFO(rclcpp::get_logger(PACKAGE_NAME), "Robot End-Effector Position:");
  RCLCPP_INFO(rclcpp::get_logger(PACKAGE_NAME), "Position: x=%.3f, y=%.3f, z=%.3f",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);
  RCLCPP_INFO(rclcpp::get_logger(PACKAGE_NAME), "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
              current_pose.pose.orientation.x,
              current_pose.pose.orientation.y,
              current_pose.pose.orientation.z,
              current_pose.pose.orientation.w);

  // Get the current robot state
  moveit::core::RobotStatePtr current_state = move_group->getCurrentState();

  // You can also get joint values if needed
  std::vector<double> joint_values;
  const moveit::core::JointModelGroup* joint_model_group = 
      current_state->getJointModelGroup(move_group->getName());
  current_state->copyJointGroupPositions(joint_model_group, joint_values);

  RCLCPP_INFO(rclcpp::get_logger(PACKAGE_NAME), "Joint Values:");
  for (size_t i = 0; i < joint_values.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger(PACKAGE_NAME), "Joint %zu: %.3f", i, joint_values[i]);
  }
}


int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    PACKAGE_NAME,
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger(PACKAGE_NAME);

  // State monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, MOVE_GROUP_NAME);

  // Get the current pose
  get_current_pose(&move_group_interface);

  // Set a target Pose
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Get new pose
  get_current_pose(&move_group_interface);

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}