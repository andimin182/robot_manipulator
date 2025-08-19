# Robot Manipulator

## Installation
- Clone the repo via *git clone  https://github.com/andimin182/robot_manipulator.git*
- Modify the docker-compose-humble.yaml file by changing the path to the tuto_ws in the *volumes* section
- Build the docker compose file with: *docker-compose -f docker-compose-humble.yaml build*
- Run a container from the image with: *docker-compose -f docker-compose-humble.yaml up*
- Connect to the container with: *docker exec --privileged -it ros2-humble-tuto-container bash*
- Run a *colcon build* from the ros_ws folder to build the packages
- Source the setup.bash: *source install/setup.bash*
- Ready to use the packages