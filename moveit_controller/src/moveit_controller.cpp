#include <rclcpp/rclcpp.hpp>
#include "moveit_controller/moveit_controller.hpp"


int main(int argc, char* argv[])
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

  // Create the MoveIt Controller
  MoveItController moveit_controller(node);

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

  std::vector<Point2D> path = moveit_controller.generateEightShapedPath(
    current_pose.pose.position.x + 0.2,
    current_pose.pose.position.y + 0.2,
    0.5,  // Path length
    50    // Number of points
  );

  RCLCPP_INFO(logger, "Generated path:");
  for (const auto& point : path)
  {
    RCLCPP_INFO(logger, "Point: x=%.3f, y=%.3f", point.x, point.y);
  }

  for (const auto& point : path)
  {
    auto const target_pose = moveit_controller.create_target_pose(
      current_pose.pose.orientation.w,
      current_pose.pose.orientation.x,
      current_pose.pose.orientation.y,
      current_pose.pose.orientation.z,
      point.x,
      point.y,
      0.2 // arbitrary z value
    );
    moveit_controller.setPoseTarget(target_pose);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}