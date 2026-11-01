#!/usr/bin/env python3

import random
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


GRIPPER_OPEN_LIM = 0.02
GRIPPER_CLOSE_LIM = -0.02
GRIPPER_STEP = -0.001
GRIPPER_SIZE_THRESHOLD = -0.004


# Joint-space waypoints from the coursework.
pick_poses = [
    [0, 0.00017, 0.00136, -0.0031],
    [-1.3358950947048042e-05, -0.1061793044936232,
     0.6570420615335268, 0.35271669927253313],
]

place_left_poses = [
    [0, 0.00017, 0.00136, -0.0031],
    [1.5992864912340499, 8.415907426146138e-05,
     1.5226615549514953e-06, 5.400218179825652e-05],
    [1.5992663386140384, 0.35269513249348816,
     0.19473188525619556, 0.08812983289545956],
]

place_right_poses = [
    [0, 0.00017, 0.00136, -0.0031],
    [-1.5992864912340499, 8.415907426146138e-05,
     1.5226615549514953e-06, 5.400218179825652e-05],
    [-1.5992663386140384, 0.35269513249348816,
     0.19473188525619556, 0.08812983289545956],
]


def build_moveit_config():
    """Build the MoveIt configuration used by this challenge."""
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="open_manipulator_x",
            package_name="open_manipulator_moveit_config",
        )
        .robot_description_semantic(
            "config/open_manipulator_x/open_manipulator_x.srdf"
        )
        .joint_limits("config/open_manipulator_x/joint_limits.yaml")
        .trajectory_execution("config/open_manipulator_x/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    config_dict = moveit_config.to_dict()
    config_dict["planning_pipelines"] = {"pipeline_names": ["ompl"]}
    config_dict["plan_request_params"] = {
        "planning_pipeline": "ompl",
        "max_velocity_scaling_factor": 0.1,
        "max_acceleration_scaling_factor": 0.1,
    }
    return config_dict


class PickAndPlace:
    """Wrapper for MoveIt planning, gripper control and scene objects."""

    def __init__(self, node):
        print("========= Initialise pick and place system =========", flush=True)
        self.latest_joint_state = None

        # This class creates its own MoveItPy instance.
        self.moveit = MoveItPy(
            node_name="moveit_py",
            config_dict=build_moveit_config(),
        )
        self.robot_model = self.moveit.get_robot_model()
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()

        self.joint_state_subscription = node.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        # Allow MoveIt 2 time to build the initial planning scene.
        time.sleep(1.0)

    def _joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def _plan_and_execute(self, group_name, joint_values):
        """Plan and execute a joint-space goal using the Jazzy MoveItPy API."""
        planning_component = self.moveit.get_planning_component(group_name)
        goal_state = RobotState(self.robot_model)
        goal_state.set_joint_group_positions(
            group_name,
            np.array(joint_values, dtype=float),
        )
        planning_component.set_start_state_to_current_state()
        planning_component.set_goal_state(robot_state=goal_state)

        print(f"Planning for {group_name}: {joint_values}", flush=True)
        moveit_plan = planning_component.plan()

        if not moveit_plan:
            print(f"ERROR: Planning failed for group '{group_name}'.", flush=True)
            return False

        # This is the MoveItPy execution call used by the Jazzy branch.
        # Jazzy MoveItPy infers the planning group from the trajectory.
        # Keep the same execution call as in your original working file.
        self.moveit.execute(
            moveit_plan.trajectory,
            controllers=[],
        )
        return moveit_plan

    # Simulation helper: remove the movable block from the planning scene.
    def remove_block(self):
        collision_object = CollisionObject(
            header=Header(frame_id="world"),
            id="block",
            operation=CollisionObject.REMOVE,
        )
        with self.planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(collision_object)
            scene.current_state.update()

    # Simulation helper: add a randomly sized block in the pick area.
    def add_block(self, box_size):
        collision_object = CollisionObject(
            header=Header(frame_id="world"),
            id="block",
            operation=CollisionObject.ADD,
            primitives=[
                SolidPrimitive(
                    type=SolidPrimitive.BOX,
                    dimensions=[box_size] * 3,
                )
            ],
            primitive_poses=[
                Pose(
                    position=Point(
                        x=0.2,
                        y=0.0,
                        z=box_size / 2.0,
                    ),
                    orientation=Quaternion(w=1.0),
                )
            ],
        )
        with self.planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(collision_object)
            scene.current_state.update()

    # Add one static cube to check that scene objects appear in RViz.
    def add_test_box(self):
        collision_object = CollisionObject()
        collision_object.header = Header(frame_id="world")
        collision_object.id = "scene_test_box"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.04, 0.04, 0.04]  # 4 cm cube

        box_pose = Pose()
        box_pose.position = Point(x=0.0, y=0.3, z=0.026)
        box_pose.orientation = Quaternion(w=1.0)

        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [box_pose]
        collision_object.operation = CollisionObject.ADD

        with self.planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(collision_object)
            scene.current_state.update()

        print("Test box added to the MoveIt planning scene.", flush=True)

    def remove_test_box(self):
        """Remove the static test cube from the planning scene."""
        collision_object = CollisionObject(
            header=Header(frame_id="world"),
            id="scene_test_box",
            operation=CollisionObject.REMOVE,
        )
        with self.planning_scene_monitor.read_write() as scene:
            scene.apply_collision_object(collision_object)
            scene.current_state.update()
        print("Test box removed from the MoveIt planning scene.", flush=True)

    # In this RViz-only simulation, attaching/detaching represents grasping.
    def attach_block(self):
        attached_collision_object = AttachedCollisionObject(
            link_name="link5",
            object=CollisionObject(
                id="block",
                operation=CollisionObject.ADD,
            ),
            touch_links=[
                "link5",
                "gripper_left_link",
                "gripper_right_link",
            ],
        )
        with self.planning_scene_monitor.read_write() as scene:
            scene.process_attached_collision_object(attached_collision_object)
            scene.current_state.update()

    def detach_block(self):
        attached_collision_object = AttachedCollisionObject(
            link_name="link5",
            object=CollisionObject(
                id="block",
                operation=CollisionObject.REMOVE,
            ),
        )
        with self.planning_scene_monitor.read_write() as scene:
            scene.process_attached_collision_object(attached_collision_object)
            scene.current_state.update()

    def grip(self):
        # Close in small steps, as in the original coursework implementation.
        for value in np.arange(
            GRIPPER_OPEN_LIM,
            GRIPPER_CLOSE_LIM,
            GRIPPER_STEP,
        ):
            self._plan_and_execute("gripper", [float(value), float(value)])

    def release(self):
        # Open in small steps to avoid large gripper commands.
        for value in np.arange(
            GRIPPER_CLOSE_LIM,
            GRIPPER_OPEN_LIM,
            -GRIPPER_STEP,
        ):
            self._plan_and_execute("gripper", [float(value), float(value)])

    def execute_move_sequence(self, sequence, name):
        """Execute each waypoint; stop if planning or execution fails."""
        print(f"========= Running {name} sequence =========", flush=True)
        for index, pose in enumerate(sequence):
            print(f"Goal pose {index}: {pose}", flush=True)
            result = self._plan_and_execute("arm", pose)
            if not result:
                print(
                    f"WARNING: {name} sequence stopped at pose {index}.",
                    flush=True,
                )
                return False
        return True

    def inspect_object(self):
        """Infer block size from the measured left gripper joint position."""
        if self.latest_joint_state is None:
            raise RuntimeError("No joint state received; cannot inspect the block.")

        try:
            joint_index = self.latest_joint_state.name.index("gripper_left_joint")
        except ValueError as exc:
            raise RuntimeError(
                "Joint 'gripper_left_joint' is missing from /joint_states."
            ) from exc

        gripper_position = self.latest_joint_state.position[joint_index]
        if gripper_position < GRIPPER_SIZE_THRESHOLD:
            return 0  # small block
        return 1  # large block


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("run_challenge")
    node.declare_parameter("sim", True)
    sim_mode = node.get_parameter("sim").value

    controller = None
    try:
        controller = PickAndPlace(node)
        attempts = 0

        # Wait for a joint-state message before commanding the gripper.
        print("Waiting for /joint_states...", flush=True)
        while rclpy.ok() and controller.latest_joint_state is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        if not rclpy.ok():
            return

        # Add the static test cube once at startup, not after every attempt.
        if sim_mode:
            controller.add_test_box()

        # Start with the gripper open.
        controller.release()

        while rclpy.ok():
            print(
                f"Attempt {attempts}. Press Enter to attempt pick and place "
                "(or enter q to quit): ",
                flush=True,
            )
            user_input = input().strip().lower()
            if user_input == "q":
                break

            if sim_mode:
                controller.remove_block()
                controller.add_block(random.choice((0.0225, 0.045)))

            # Pick the block and close the gripper.
            if not controller.execute_move_sequence(pick_poses, "pick"):
                print("Pick sequence failed; skipping this attempt.", flush=True)
                attempts += 1
                continue

            controller.grip()
            rclpy.spin_once(node, timeout_sec=0.1)
            object_type = controller.inspect_object()

            if sim_mode:
                controller.attach_block()

            # Sort small blocks left and large blocks right.
            if object_type == 0:
                placement_ok = controller.execute_move_sequence(
                    place_left_poses,
                    "place left",
                )
            elif object_type == 1:
                placement_ok = controller.execute_move_sequence(
                    place_right_poses,
                    "place right",
                )
            else:
                print(f"Unknown object type: {object_type}", flush=True)
                placement_ok = False

            if not placement_ok:
                print(
                    "Placement sequence failed. The block may still be attached; "
                    "inspect the scene before continuing.",
                    flush=True,
                )
                break

            if sim_mode:
                controller.detach_block()
            controller.release()
            attempts += 1

    except (KeyboardInterrupt, EOFError):
        print("Challenge interrupted; shutting down.", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
