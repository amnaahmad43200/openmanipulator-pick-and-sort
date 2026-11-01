# OpenManipulator Lab

`openmanipulatorlab` is a ROS 2 `ament_python` package for an OpenManipulator pick-and-sort challenge using MoveIt 2.

The repository currently contains:

- `openmanipulatorlab/run_challenge.py`: the challenge node
- `launch/run_challenge.launch.py`: bringup and challenge launch file
- `models/`: model assets installed with the package

The challenge script is intentionally incomplete for teaching use:

- the waypoint lists are still blank
- `execute_move_sequence()` still contains the missing planning and execution lines
- `inspect_object()` still returns a random bin until students replace it

## Build

From a colcon workspace:

```bash
colcon build --packages-select openmanipulatorlab
source install/setup.bash
```

## Run

Bring up the robot, MoveIt, and the challenge node in simulation:

```bash
ros2 launch openmanipulatorlab run_challenge.launch.py sim:=true
```

For the real robot:

```bash
ros2 launch openmanipulatorlab run_challenge.launch.py sim:=false
```

## Dependencies

This package expects the ROS 2 OpenManipulator bringup and MoveIt configuration packages to be available in the environment, including:

- `open_manipulator_bringup`
- `open_manipulator_moveit_config`
- `moveit_py`
