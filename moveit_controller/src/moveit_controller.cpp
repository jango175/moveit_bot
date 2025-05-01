#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "moveit_controller/eight_path.h"

#define PACKAGE_NAME       "moveit_controller"
#define MOVE_GROUP_NAME    "panda_arm"
#define BASE_LINK_NAME     "panda_link0"

geometry_msgs::msg::PoseStamped get_current_pose(moveit::planning_interface::MoveGroupInterface *move_group)
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

  return current_pose;
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

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, BASE_LINK_NAME, rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup(MOVE_GROUP_NAME)](
          auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

  // Get the current pose
  geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();

  std::vector<Point2D> path = generate_eight_shaped_path(
    current_pose.pose.position.x+0.2,
    current_pose.pose.position.y+0.2,
    0.5,  // Path length
    12   // Number of points
  );

  RCLCPP_INFO(logger, "Generated path:");
  for (const auto& point : path) {
    RCLCPP_INFO(logger, "Point: x=%.3f, y=%.3f", point.x, point.y);
  }

  for (const auto& point : path) {
    // Set a target pose
    auto const target_pose = [&current_pose, &point]{
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = current_pose.pose.orientation.w;
      msg.orientation.x = current_pose.pose.orientation.x;
      msg.orientation.y = current_pose.pose.orientation.y;
      msg.orientation.z = current_pose.pose.orientation.z;
      msg.position.x = point.x;
      msg.position.y = point.y;
      msg.position.z = 0.2;
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
      // Draw the trajectory
      draw_trajectory_tool_path(plan.trajectory);
      moveit_visual_tools.trigger();

      move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }
  }

  // Get new pose
  get_current_pose(&move_group_interface);

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}