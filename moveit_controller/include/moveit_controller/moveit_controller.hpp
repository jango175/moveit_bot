#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#define PACKAGE_NAME       "moveit_controller"
#define MOVE_GROUP_NAME    "panda_arm"
#define BASE_LINK_NAME     "panda_link0"


// 2D point structure
struct Point2D
{
  double x;
  double y;
};


/**
 * @brief MoveItController class
 */
class MoveItController
{
private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Logger logger;
  moveit::planning_interface::MoveGroupInterface move_group_interface;
  moveit_visual_tools::MoveItVisualTools moveit_visual_tools;

public:
  /**
   * @brief Constructor
   * @param node Shared pointer to the ROS node
   */
  MoveItController(rclcpp::Node::SharedPtr node)
    : node(node),
    logger(rclcpp::get_logger(PACKAGE_NAME)),
    move_group_interface(this->node, MOVE_GROUP_NAME),
    moveit_visual_tools(this->node, BASE_LINK_NAME, rviz_visual_tools::RVIZ_MARKER_TOPIC,
                        this->move_group_interface.getRobotModel())
  {
    this->clearMarkers();
  }


  /**
   * @brief Clear all markers in RViz
   */
  void clearMarkers()
  {
    this->moveit_visual_tools.deleteAllMarkers();
    this->moveit_visual_tools.loadRemoteControl();
  }


  /**
   * @brief Get the current pose of the robot
   * 
   * @return Current pose of the robot
   */
  geometry_msgs::msg::PoseStamped getCurrentPose()
  {
    geometry_msgs::msg::PoseStamped current_pose = this->move_group_interface.getCurrentPose();

    return current_pose;
  }


  /**
   * @brief Get the current state of the robot
   * 
   * @return Current state of the robot
   */
  moveit::core::RobotStatePtr getCurrentState()
  {
    moveit::core::RobotStatePtr current_state = this->move_group_interface.getCurrentState();

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

    const moveit::core::JointModelGroup* joint_model_group = 
      current_state->getJointModelGroup(this->move_group_interface.getName());
    current_state->copyJointGroupPositions(joint_model_group, joint_values);

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
  geometry_msgs::msg::Pose create_target_pose(
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
    this->move_group_interface.setPoseTarget(target_pose);

    auto const draw_trajectory_tool_path = [this,
      jmg = this->move_group_interface.getRobotModel()->getJointModelGroup(MOVE_GROUP_NAME)
      ](auto const trajectory){
      this->moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };

    // Create a plan to that target pose
    auto const [success, plan] = [this]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(this->move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
      // Draw the trajectory
      draw_trajectory_tool_path(plan.trajectory);
      this->moveit_visual_tools.trigger();

      this->move_group_interface.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(this->logger, "Planning failed!");
    }
  }


  /**
   * @brief Generate a figure-8 shaped path
   * 
   * @param start_x Starting x coordinate
   * @param start_y Starting y coordinate
   * @param path_length Length of the path
   * @param num_points Number of points in the path
   * 
   * @return Vector of points representing the figure-8 path
   */
  std::vector<Point2D> generateEightShapedPath(
    double start_x, 
    double start_y, 
    double path_length,
    int num_points = 100)
  {
    std::vector<Point2D> path;

    // Calculate the radius of each circle in the figure-8
    // The total length of a figure-8 made of two circles is approximately 4*r
    double radius = path_length / 4.0;
    Point2D point;

    // Generate points along the figure-8 path
    for (int i = 0; i < num_points; ++i)
    {
      double t = 2.0 * M_PI * i / num_points;

      // Parametric equation for a figure-8
      // Using lemniscate of Gerono: x = cos(t), y = sin(t)*cos(t)
      // Scaled and translated to the desired position and size
      point.x = start_x + radius * sin(t);
      point.y = start_y + radius * sin(t) * cos(t);

      path.push_back(point);
    }

    // Back to the starting point
    point.x = start_x;
    point.y = start_y;
    path.push_back(point);

    return path;
  }


  /**
   * @brief Destructor
   */
  ~MoveItController()
  {
    this->clearMarkers();
    this->move_group_interface.stop();
    this->move_group_interface.clearPoseTargets();
    this->move_group_interface.clearPathConstraints();
  }
};
