# prbt_robot_moveit_config
ROS2 moveit config for prbt robot

This is a temporary repository, that contains the moveit2 configuration for the ros2_canopen powered PRBT robot.
The repo will be merged with ipa-cmh/prbt_robot in time.

## Usage
Careful, dependencies are not well defined or checked.
You have to add rosindustrial/ros2_canopen and ipa-cmh/prbt_robot to your workspace and build them before using this.

**Virtual CAN emulation**

Start the canopen stack:
```
ros2 launch prbt_robot_moveit_config vcan.launch.py
```

Initialise the joints and set position mode for each joint:
```
ros2 service call prbt_joint_1_controller/init std_srvs/srv/Trigger
ros2 service call prbt_joint_1_controller/position_mode std_srvs/srv/Trigger
```

Now you can start planning and executing trajectories with moveit/rviz.
This uses fake cia402_slave via vcan0, so do not forget to setup vcan0.
