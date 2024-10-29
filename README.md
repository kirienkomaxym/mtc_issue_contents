# mtc_issue_contents
Repository containing context for moveit task constructor issue
## Install
```
cd ../mtc_issue_contents
colcon build
```
## Package Requirements
The suitable docker environment is isaac-ros-docker. It has all required packages, however here are the versions of packages required:
* ROS2 distro: HUMBLE
* ur_robot_driver: 2.2.15
* moveit2: 2.2.5
* moveit_task_constructor_core: 0.1.3
* robotiq_description: 0.0.1

## Launches needed
Launch URSIM (ur robot emulator )with script:
`bash <path_to>/launch_ursim.sh`
(docker configured in non-sudo mode required, however you can just exec it with sudo)

Run the ur-robot-driver launch.py
```
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=<ursim_ip> \
    controllers_file:=<path_to>/ur_robotiq_ur_robot_driver_config/config/ros2_controllers.yaml \
    description_package:=ur_robotiq_ur_robot_driver_config \
    description_file:=ur5e_robotiq_2f_85_urdf.xacro \
    launch_rviz:=false
```
Run the pick & place 
```
ros2 launch mercurio_moveit_task_constructor mtc.launch.py exe:=pick_place_demo \
    ur_type:=ur5e \
    launch_rviz:=true \
    description_package:=ur_robotiq_ur_robot_driver_config \
    description_file:=ur5e_robotiq_2f_85_urdf.xacro \
    moveit_config_package:=ur_robotiq_ur_robot_driver_config
```
Do the MTC service call with:
```
ros2 service call /execute_pick_place moveit_task_constructor_msgs/srv/PickPlace "{
  object_pose: {
    position: {x: 0.5, y: -0.25, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  place_pose: {
    position: {x: 0.6, y: -0.15, z: -0.1},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```
## Extras
* mtc_logs.txt – logs from the Moveit Task Constructor
* ur_robot_driver.txt - logs from the ur robot driver




