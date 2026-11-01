# OpenManipulator Pick-and-Sort

A ROS 2 package that makes an [OpenManipulator-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/) arm pick up a block, **measure its size with the gripper**, and sort it into a left or right bin. Motion is planned with MoveIt 2 (`moveit_py`) and runs against mock hardware in RViz.

![Demo of the arm sorting small and large blocks]([docs/demo.gif](https://github.com/amnaahmad43200/openmanipulator-pick-and-sort/blob/main/rviz_block_movement_short.mp4.gif))

## What it does

For each attempt the arm:

1. Spawns a block in the planning scene (simulation) — randomly small (2.25 cm) or large (4.5 cm).
2. Moves to the pick pose and closes the gripper.
3. Reads the gripper joint position: if the fingers closed past a threshold the block is **small**, otherwise **large**.
4. Carries the block to the **left bin (small)** or **right bin (large)** and opens the gripper.

```
Spawned large block, detected large.
...
Done: 6 attempt(s), 3 sorted left (small), 3 sorted right (large).
```

## How it works

```
  run_challenge.launch.py                       run_challenge  (this package's node)
 ┌───────────────────────────────┐             ┌─────────────────────────────────────┐
 │ open_manipulator_bringup      │             │ config/challenge.yaml  ──► config.py │
 │   ros2_control + controllers  │◄── plans ───│ MoveItPy: plan + execute joint goals │
 │   robot_state_publisher       │   executes  │ planning scene: block, attach/detach │
 │ open_manipulator_moveit_config│             │ /joint_states ──► block-size check   │
 │   move_group + RViz           │ ──────────► └─────────────────────────────────────┘
 └───────────────────────────────┘  /joint_states
```

- **The launch file** starts the robot side: controllers, MoveIt and RViz. With `sim:=true` it uses ROS 2 mock hardware, which simply echoes commanded joint positions back as measured ones.
- **The node** (`run_challenge.py`) runs the pick-and-sort loop. It plans joint-space goals through `MoveItPy`, manages the block in the planning scene, and decides which bin to use from `/joint_states`.
- **Why sorting needs a simulated gripper stop:** mock hardware has no physics, so nothing would stop the fingers and every block would measure as "small". In simulation the gripper therefore stops at the width of the block that was spawned (see [Known limitations](#known-limitations)). On real hardware the block itself stops the fingers.
- **Configuration** (poses, gripper limits, block sizes, planning speed) lives in `config/challenge.yaml`, loaded and validated by `config.py`. The right-bin poses are derived by mirroring the left-bin poses (flipping the base rotation).

## Requirements

- Ubuntu with **ROS 2 Jazzy** (the code uses the Jazzy `MoveItPy` API)
- MoveIt 2 with `moveit_py`
- The ROBOTIS OpenManipulator ROS 2 packages: `open_manipulator_bringup` and `open_manipulator_moveit_config`
- `python3-numpy`, `python3-yaml`

## Build

From the `src` folder of a colcon workspace:

```bash
git clone https://github.com/amnaahmad43200/openmanipulator-pick-and-sort.git
cd ..
colcon build --packages-select openmanipulatorlab --symlink-install
source install/setup.bash
```

## Run

Use two terminals (the node waits for keyboard input, so it can't be started from the launch file).

**Terminal 1 — robot, MoveIt and RViz:**

```bash
ros2 launch openmanipulatorlab run_challenge.launch.py sim:=true
```

**Terminal 2 — the challenge:**

```bash
# Interactive: press Enter for each attempt, q to quit
ros2 run openmanipulatorlab run_challenge --ros-args -p sim:=true

# Automatic: run 6 attempts, then print a summary
ros2 run openmanipulatorlab run_challenge --ros-args -p sim:=true -p attempts:=6
```

### Node parameters

| Parameter | Default | Meaning |
|---|---|---|
| `sim` | `true` | Add and sort simulated blocks. Set `false` on a real arm, where you place the block yourself. |
| `attempts` | `0` | `0` = interactive (Enter per attempt). `N > 0` = run N attempts automatically. |
| `config_file` | *(installed default)* | Path to an alternative YAML configuration. |

Set `sim` consistently for both the launch file and the node.

## Configuration

`config/challenge.yaml` holds everything you might want to tune without touching code:

| Section | Contents |
|---|---|
| `gripper` | open and close limits, closing step size, and the size threshold that separates small from large |
| `planning` | planning pipeline and velocity/acceleration scaling (10% by default, for safety) |
| `blocks` | small and large block edge lengths, and where they spawn |
| `poses` | joint-space waypoints for picking and for the left bin |

`config.py` validates the file on load (4 joints per pose, threshold inside the gripper range, `small < large`) and fails with a clear message otherwise.

## Tests

The configuration logic is tested without ROS:

```bash
python3 -m pytest test -q
```

## Project layout

```
.
├── config/challenge.yaml          # poses, gripper, block and planning settings
├── launch/run_challenge.launch.py # robot bringup + MoveIt + RViz
├── openmanipulatorlab/
│   ├── config.py                  # load/validate config, simulated gripper stop
│   └── run_challenge.py           # the pick-and-sort node
├── models/                        # block.urdf, tray.sdf, pickandplace.scene (reference assets, not loaded by the code)
├── test/test_config.py            # unit tests
├── docs/demo.gif
├── package.xml, setup.py, setup.cfg, resource/
```

## Known limitations

- **Simulated gripper geometry is an assumption.** The simulation assumes the finger gap changes linearly with the joint position, with an offset chosen so the size threshold sits halfway between the two block sizes. It is a stand-in for physics, not a measurement of the real gripper.
- **Developed and tested in simulation.** The `sim:=false` path for a real arm exists but has not been verified in this repository.
- **Segmentation fault on exit.** After all work has finished and been printed, the process crashes while `MoveItPy` is torn down. This is a known class of issue with `moveit_py` shutdown; it does not affect the sorting.
- **Slow gripper motion.** The gripper closes in 1 mm steps, each planned and executed separately, which is slow in simulation.
- **Placement failure stops the run**, leaving the block attached in the planning scene; restart the node for a clean scene.

## Acknowledgements

This project is based on the OpenManipulator pick-and-sort lab from the University of Surrey (EEE3043), which supplied the starting code, waypoints and robot setup. My additions are the simulation fix that makes sorting actually exercise both bins, the YAML configuration and validation, the automatic mode, the unit tests, and this documentation. The robot packages are by [ROBOTIS](https://github.com/ROBOTIS-GIT).

## License

Apache-2.0, as declared in `package.xml`.
