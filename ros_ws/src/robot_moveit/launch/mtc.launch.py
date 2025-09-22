from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("manipulator", 
                                         package_name="robot_moveit")
                                         .robot_description(file_path= os.path.join(get_package_share_directory("robot_simulation_pkg"), "urdf", "robot.urdf.xacro"))
                                         .robot_description_semantic(file_path="config/robot.srdf")
                                         .trajectory_execution(file_path="config/moveit_controllers.yaml")
                                         .robot_description_kinematics(file_path="config/kinematics.yaml") 
                                         .to_moveit_configs()
                                         
                                         )

    # MTC Demo node
    pick_place_demo = Node(
        package="robot_moveit",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription(
        [
        pick_place_demo
        ]
        )