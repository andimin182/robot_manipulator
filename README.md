# Robot Manipulator
A 3R manipulator with a gripper.

## Installation
- Clone the repo via *git clone  https://github.com/andimin182/robot_manipulator.git*
- Modify the docker-compose-humble.yaml file by changing the path to the tuto_ws in the *volumes* section
- Build the docker compose file with: *docker-compose -f docker-compose-humble.yaml build*
- Run a container from the image with: *docker-compose -f docker-compose-humble.yaml up*
- Connect to the container with: *docker exec --privileged -it ros2-humble-tuto-container bash*
- Run a *colcon build* from the ros_ws folder to build the packages
- Source the setup.bash: *source install/setup.bash*
- Ready to use the packages

## Packages
- robot_controller_pkg: pkg to load the ros2_control controllers for the manipulator
- robot_moveit: pkg that uses Moveit2 to create trajectories. NOTE: install the Moveit Task Constructor plugin to use the mtc_node
- robot_msgs: pkg to iinclude the custom designed msgs, services and actions
- robot_simulation_pkg: pkg to describe the manipulator kinematics
- robot_utils: pkg including the custom functions
- tuto_cmake_pkg: pkg with nodes in C++
- tuto_pkg: pkg with nodes in Python

## Functionalities
- **Pick and Place task**: 

It implementes a Pick and Place task for the manipulator using the Moveit Task Constructor (MTC).
To use it, launch the manipulator rviz visualization with fake controllers:
$ros2 launch robot_moveit moveit_fake.launch.py
Then, launch the node creating the task:
$ros2 launch robot_moveit mtc_node

- **Gazebo simulation with Moveit2**

Run the gazebo simulation with ros2_control controllers and Moveit2 to generate trajectories and execute them:
0. Change the xacro param use_face_control to false in robot_ros2_control.xacro
1. $ros2 launch robot_simulation_pkg gazebo.launch.py
2. $ros2 launch robot_controller_pkg controller.launch.py
3. $ros2 launch robot_moveit moveit.launch.py
