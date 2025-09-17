from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    robot_description = ParameterValue(Command(
        [
            "xacro ", 
            os.path.join(get_package_share_directory("robot_simulation_pkg"), "urdf", "robot.urdf.xacro")
        ]
    ),
        value_type=str,
        )
    # Moveit node
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="False",
        description="Flag to use the simulation time"
    )

    is_sim = LaunchConfiguration("is_sim")

    moveit_config = (
        MoveItConfigsBuilder("manipulator", 
                                         package_name="robot_moveit")
                                         .robot_description(file_path= os.path.join(get_package_share_directory("robot_simulation_pkg"), "urdf", "robot.urdf.xacro"))
                                         .robot_description_semantic(file_path="config/robot.srdf")
                                         .trajectory_execution(file_path="config/moveit_controllers.yaml")
                                         .to_moveit_configs()
                                         
                                         )
    
    moveit_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), 
                    {"use_sim_time": is_sim},
                    {"publish_robot_description_semantic": True}],

        arguments= [
            "--ros-args", "--log-level", "info"
        ]
    )
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Ros2 control node
    robot_controllers = os.path.join(get_package_share_directory("robot_controller_pkg"), "config", "robot_controllers.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            robot_controllers
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file", robot_controllers
        ]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file", robot_controllers
        ]
    )

    grip_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file", robot_controllers
        ]
    )

    rviz_config = os.path.join(get_package_share_directory("robot_moveit"), "config", "moveit_fake.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d", rviz_config
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ]

    )
    return LaunchDescription([
        is_sim_arg,
        robot_state_publisher_node,
        moveit_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        grip_controller_spawner,
        rviz_node,
    ])