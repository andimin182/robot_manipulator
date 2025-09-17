from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # First launch the robot state publisher to see the robot_description
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value = os.path.join(get_package_share_directory("robot_simulation_pkg"), "urdf", "robot.urdf.xacro"),
        description=" Absolute path to URDF model file"
        )
    
    # Full plain URDF robot model (converted from xacro)
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")])

    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Launch the joint state pub gui that allows to move the robot
    joint_state_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # Finally launch Rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("robot_simulation_pkg"), "rviz", "display_robot.rviz" )]
    )
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_pub_gui,
        rviz_node
    ])