#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#define MOVE_GROUP_NAME    "panda_arm"
#define BASE_LINK_NAME     "panda_link0"


// 3D point structure
struct Point3D
{
  double qw;
  double qx;
  double qy;
  double qz;

  double x;
  double y;
  double z;
};


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
  MoveItController(rclcpp::Node::SharedPtr node)
    : node_(node),
    logger_(rclcpp::get_logger(this->node_->get_name())),
    move_group_interface_(this->node_, MOVE_GROUP_NAME),
    robot_model_(this->move_group_interface_.getRobotModel()),
    joint_model_group_(this->robot_model_->getJointModelGroup(this->move_group_interface_.getName())),
    moveit_visual_tools_(this->node_, BASE_LINK_NAME, rviz_visual_tools::RVIZ_MARKER_TOPIC,
                        this->robot_model_)
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
   * @brief Set the target pose for the robot and execute the plan
   * 
   * @param target_pose Target pose for the robot
   */
  void setPoseTarget(const geometry_msgs::msg::Pose& target_pose)
  {
    this->move_group_interface_.setPoseTarget(target_pose);

    auto const draw_trajectory_tool_path = [this, jmg = this->joint_model_group_](auto const trajectory){
      this->moveit_visual_tools_.publishTrajectoryLine(trajectory, jmg);
    };

    // Create a plan to that target pose
    auto const [success, plan] = [this]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(this->move_group_interface_.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
      // Draw the trajectory
      draw_trajectory_tool_path(plan.trajectory);
      this->moveit_visual_tools_.trigger();

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
   * @param start_point Starting point of the path
   * @param path_length Length of the path
   * @param num_points Number of points in the path
   * 
   * @return Vector of points representing the figure-8 path
   */
  std::vector<Point3D> generateEightShapedPath(Point3D start_point, double path_length, int num_points = 100)
  {
    std::vector<Point3D> path;

    // Calculate the radius of each circle in the figure-8
    // The total length of a figure-8 made of two circles is approximately 4*r
    double radius = path_length / 4.0;
    Point3D point;

    // Generate points along the figure-8 path
    for (int i = 0; i < num_points; ++i)
    {
      double t = 2.0 * M_PI * i / num_points;

      point = start_point;

      // Parametric equation for a figure-8
      // Using lemniscate of Gerono: x = cos(t), y = sin(t)*cos(t)
      // Scaled and translated to the desired position and size
      point.x += radius * sin(t);
      point.y += radius * sin(t) * cos(t);

      path.push_back(point);
    }

    // Back to the starting point
    path.push_back(start_point);

    return path;
  }


  robot_trajectory::RobotTrajectory createTrajectoryFromPoints(const std::vector<Point3D>& waypoints)
  {
    // Create robot state and trajectory objects
    moveit::core::RobotState robot_state(this->robot_model_);
    robot_trajectory::RobotTrajectory trajectory(this->robot_model_, this->joint_model_group_);

    // Add each waypoint to the trajectory
    for (const auto& waypoint : waypoints)
    {
      // Create pose target from waypoint
      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = waypoint.x;
      target_pose.position.y = waypoint.y;
      target_pose.position.z = waypoint.z;

      // Use the quaternion from the waypoint
      target_pose.orientation.w = waypoint.qw;
      target_pose.orientation.x = waypoint.qx;
      target_pose.orientation.y = waypoint.qy;
      target_pose.orientation.z = waypoint.qz;

      // Compute IK to get joint positions for this Cartesian pose
      bool found_ik = robot_state.setFromIK(this->joint_model_group_, target_pose);
      
      if (!found_ik)
      {
        RCLCPP_WARN(this->logger_, "IK solution not found for waypoint (%f, %f, %f)", 
                    waypoint.x, waypoint.y, waypoint.z);
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
    }
    else
    {
      RCLCPP_ERROR(this->logger_, "Failed to parameterize trajectory!");
    }

    return trajectory;
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
