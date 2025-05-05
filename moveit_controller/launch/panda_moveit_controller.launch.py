from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define the MoveGroupInterface name
    arg = DeclareLaunchArgument(
        'move_group_interface_name',
        default_value='panda_arm',
        description='The name of the MoveGroupInterface to be used'
    )

    # Set MoveIt configuration
    moveit_config = MoveItConfigsBuilder(robot_name="panda", package_name="panda_moveit_config").to_moveit_configs()

    # Define moveit_controller node
    moveit_controller_node = Node(
        package="moveit_controller",
        executable="moveit_controller",
        output="screen",
        parameters=[
            {'move_group_interface_name': LaunchConfiguration('move_group_interface_name')},
            moveit_config.robot_description, # Load URDF
            moveit_config.robot_description_semantic, # Load SRDF
            moveit_config.robot_description_kinematics, # Load kinematics.yaml
            moveit_config.joint_limits, # Load joint_limits.yaml
        ],
    )

    return LaunchDescription([arg, moveit_controller_node])
