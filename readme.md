# Walking robot control

This package is used in parallel with [robot_walker_moveit](https://github.com/Michael-Law/robot_walker_moveit.git). It controls two kinematics solver; direct kinematics and inverse kinematics.

For direct kinematis use:
```sh
roslaunch robot_walker_control directKin.launch
```

FOr inverse kinematics uses:

```sh
roslaunch udm_hand_control indirect.launch
```
