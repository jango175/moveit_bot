#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_state/conversions.hpp>


/**
 * @brief MoveItController class
 */
class MoveItController
{
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  const moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_;
  moveit_visual_tools::MoveItVisualTools moveit_visual_tools_;

public:
  /**
   * @brief Constructor
   * 
   * @param node_ Shared pointer to the ROS node
   */
  MoveItController(rclcpp::Node::SharedPtr node, std::string move_group_name)
    : node_(node),
    logger_(rclcpp::get_logger(this->node_->get_name())),
    move_group_interface_(this->node_, move_group_name),
    robot_model_(this->move_group_interface_.getRobotModel()),
    joint_model_group_(this->robot_model_->getJointModelGroup(this->move_group_interface_.getName())),
    moveit_visual_tools_(this->node_, this->move_group_interface_.getPlanningFrame(),
                         rviz_visual_tools::RVIZ_MARKER_TOPIC, this->robot_model_)
  {
    this->clearMarkers();
  }


  /**
   * @brief Clear all markers in RViz
   */
  void clearMarkers()
  {
    this->moveit_visual_tools_.deleteAllMarkers();
    this->moveit_visual_tools_.loadRemoteControl();
  }


  /**
   * @brief Get the current pose of the robot
   * 
   * @return Current pose of the robot
   */
  geometry_msgs::msg::PoseStamped getCurrentPose()
  {
    geometry_msgs::msg::PoseStamped current_pose = this->move_group_interface_.getCurrentPose();

    return current_pose;
  }


  /**
   * @brief Get the current state of the robot
   * 
   * @return Current state of the robot
   */
  moveit::core::RobotStatePtr getCurrentState()
  {
    moveit::core::RobotStatePtr current_state = this->move_group_interface_.getCurrentState();

    return current_state;
  }


  /**
   * @brief Get the current joint values of the robot
   * 
   * @return Current joint values of the robot
   */
  std::vector<double> getJointValues()
  {
    moveit::core::RobotStatePtr current_state = this->getCurrentState();

    std::vector<double> joint_values;

    current_state->copyJointGroupPositions(this->joint_model_group_, joint_values);

    return joint_values;
  }


  /**
   * @brief Create a target pose for the robot
   * 
   * @param qw Quaternion w component
   * @param qx Quaternion x component
   * @param qy Quaternion y component
   * @param qz Quaternion z component
   * @param x Position x coordinate
   * @param y Position y coordinate
   * @param z Position z coordinate
   * 
   * @return Target pose for the robot
   */
  geometry_msgs::msg::Pose createTargetPose(
    double qw, double qx, double qy, double qz,
    double x, double y, double z)
  {
    auto const target_pose = [qw, qx, qy, qz, x, y, z]{
      geometry_msgs::msg::Pose msg;
      msg.orientation.w = qw;
      msg.orientation.x = qx;
      msg.orientation.y = qy;
      msg.orientation.z = qz;
      msg.position.x = x;
      msg.position.y = y;
      msg.position.z = z;
      return msg;
    }();

    return target_pose;
  }


  /**
   * @brief Draw the trajectory of the tool path
   * 
   * @param trajectory Robot trajectory
   */
  void draw_trajectory_tool_path(const moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    this->moveit_visual_tools_.publishTrajectoryLine(trajectory, this->joint_model_group_);
    this->moveit_visual_tools_.trigger();
  }


  /**
   * @brief Set the target pose for the robot and execute the plan
   * 
   * @param target_pose Target pose for the robot
   */
  void setPoseTarget(const geometry_msgs::msg::Pose& target_pose)
  {
    this->move_group_interface_.setPoseTarget(target_pose);

    // Create a plan to that target pose
    auto const [success, plan] = [this]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(this->move_group_interface_.plan(msg));
      return std::make_pair(ok, msg);
    }();

    if (success)
    {
      // Draw the trajectory
      this->draw_trajectory_tool_path(plan.trajectory);

      // Execute the plan
      this->move_group_interface_.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(this->logger_, "Planning failed!");
    }
  }


  /**
   * @brief Generate a figure-8 shaped path
   * 
   * @param start_point Starting pose of the path
   * @param path_length Length of the path
   * @param num_points Number of points in the path
   * 
   * @return Vector of poses representing the figure-8 path
   */
  std::vector<geometry_msgs::msg::Pose> generateEightShapedPath(
    geometry_msgs::msg::Pose start_pose,
    double path_length,
    int num_points = 100)
  {
    std::vector<geometry_msgs::msg::Pose> path;

    // Calculate the radius of each circle in the figure-8
    // The total length of a figure-8 made of two circles is approximately 4*r
    double radius = path_length / 4.0;
    geometry_msgs::msg::Pose pose;

    // Generate points along the figure-8 path
    for (int i = 0; i < num_points; ++i)
    {
      double t = 2.0 * M_PI * i / num_points;

      pose = start_pose;

      // Parametric equation for a figure-8
      // Using lemniscate of Gerono: x = cos(t), y = sin(t)*cos(t)
      // Scaled and translated to the desired position and size
      pose.position.x += radius * sin(t);
      pose.position.y += radius * sin(t) * cos(t);

      path.push_back(pose);
    }

    // Back to the starting pose
    path.push_back(start_pose);

    return path;
  }


  /**
   * @brief Set a trajectory from a set of waypoints
   * 
   * @param waypoints Vector of waypoints
   */
  void setTrajectoryTarget(const std::vector<geometry_msgs::msg::Pose>& waypoints)
  {
    // Create robot state and trajectory objects
    moveit::core::RobotState robot_state(this->robot_model_);
    robot_trajectory::RobotTrajectory trajectory(this->robot_model_, this->joint_model_group_);

    // Add the start state to the trajectory
    robot_state.setJointGroupPositions(this->joint_model_group_, this->getJointValues());
    robot_state.enforceBounds();
    trajectory.addPrefixWayPoint(robot_state, 0.1);

    // Add each waypoint to the trajectory
    for (const auto& waypoint : waypoints)
    {
      // Compute IK to get joint positions for this Cartesian pose
      bool found_ik = robot_state.setFromIK(this->joint_model_group_, waypoint);
      
      if (!found_ik)
      {
        RCLCPP_WARN(this->logger_, "IK solution not found for waypoint (%f, %f, %f)", 
                    waypoint.position.x, waypoint.position.y, waypoint.position.z);
        continue; // Skip this waypoint
      }

      // Make sure the state is valid (within joint limits)
      robot_state.enforceBounds();

      // Add this point to the trajectory
      trajectory.addSuffixWayPoint(robot_state, 0.1);
    }

    // Now apply TOTG to calculate optimal timing
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    bool success = time_parameterization.computeTimeStamps(trajectory);

    if (success)
    {
      RCLCPP_INFO(this->logger_, "Successfully generated time-optimal trajectory!");

      // Now trajectory has properly timed waypoints
      // You can extract it as a trajectory_msgs::JointTrajectory message
      moveit_msgs::msg::RobotTrajectory trajectory_msg;
      trajectory.getRobotTrajectoryMsg(trajectory_msg);

      // Print some info about the trajectory
      RCLCPP_INFO(this->logger_, "Trajectory has %zu points with total duration %.3f seconds", 
                  trajectory_msg.joint_trajectory.points.size(),
                  trajectory_msg.joint_trajectory.points.back().time_from_start.sec +
                  trajectory_msg.joint_trajectory.points.back().time_from_start.nanosec / 1e9);

      // Create a plan
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory = trajectory_msg;

      // Draw the trajectory
      this->draw_trajectory_tool_path(plan.trajectory);

      // Execute the plan
      this->move_group_interface_.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(this->logger_, "Failed to parameterize trajectory!");
    }
  }


  /**
   * @brief Destructor
   */
  ~MoveItController()
  {
    this->clearMarkers();
    this->move_group_interface_.stop();
    this->move_group_interface_.clearPoseTargets();
    this->move_group_interface_.clearPathConstraints();
  }
};
