#include <rclcpp/rclcpp.hpp>
#include "moveit_controller/moveit_controller.hpp"

#define PACKAGE_NAME    "moveit_controller"


/**
 * @brief Main function
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * 
 * @return 0 on success, -1 on failure
 */
int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    PACKAGE_NAME,
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Get the move group name from the parameter server
  std::string move_group_name;
  node->get_parameter("move_group_interface_name", move_group_name);

  // Create a ROS logger
  auto const logger = rclcpp::get_logger(node->get_name());

  // State monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt Controller
  MoveItController moveit_controller(node, move_group_name);

  // Get the current pose
  geometry_msgs::msg::PoseStamped current_pose = moveit_controller.getCurrentPose();

  // Print the current pose
  RCLCPP_INFO(logger, "Current pose:");
  RCLCPP_INFO(logger, "Position: x=%.3f, y=%.3f, z=%.3f",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);
  RCLCPP_INFO(logger, "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
              current_pose.pose.orientation.x,
              current_pose.pose.orientation.y,
              current_pose.pose.orientation.z,
              current_pose.pose.orientation.w);

  // Generate a figure-8 shaped path
  geometry_msgs::msg::Pose start_pose = moveit_controller.createTargetPose(
    current_pose.pose.orientation.w,
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    current_pose.pose.position.z
  );
  std::vector<geometry_msgs::msg::Pose> path = moveit_controller.generateEightShapedPath(start_pose, 0.1, 20);

  moveit_controller.setTrajectoryTarget(path);

  // Go from point to point
  // for (const auto& point : path)
  // {
  //   auto const target_pose = moveit_controller.createTargetPose(point.qw,
  //                                                               point.qx,
  //                                                               point.qy,
  //                                                               point.qz,
  //                                                               point.x,
  //                                                               point.y,
  //                                                               point.z);
  //   moveit_controller.setPoseTarget(target_pose);
  // }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}