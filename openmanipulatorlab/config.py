"""Load and validate the challenge configuration.

Kept free of ROS imports (except the optional default-path lookup) so it can be
unit tested without a ROS installation.
"""

import os

import yaml

PACKAGE_NAME = "openmanipulatorlab"
ARM_JOINTS = 4


def default_config_path():
    """Path of config/challenge.yaml in the installed package share directory."""
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(
        get_package_share_directory(PACKAGE_NAME), "config", "challenge.yaml"
    )


def mirror_pose(pose):
    """Mirror an arm pose to the other side by flipping the base rotation."""
    mirrored = list(pose)
    mirrored[0] = -mirrored[0]
    return mirrored


def _check_pose_list(name, poses):
    if not poses:
        raise ValueError(f"poses.{name} must contain at least one pose")
    for index, pose in enumerate(poses):
        if len(pose) != ARM_JOINTS:
            raise ValueError(
                f"poses.{name}[{index}] has {len(pose)} values, expected {ARM_JOINTS}"
            )


def load_config(path=None):
    """Read the YAML file, validate it and add the derived right-bin poses."""
    path = path or default_config_path()
    with open(path, encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle)

    gripper = cfg["gripper"]
    if not gripper["close_limit"] < gripper["size_threshold"] < gripper["open_limit"]:
        raise ValueError(
            "gripper.size_threshold must lie between close_limit and open_limit"
        )
    if gripper["step"] >= 0:
        raise ValueError("gripper.step must be negative (it is the closing step)")

    blocks = cfg["blocks"]
    if not 0 < blocks["small"] < blocks["large"]:
        raise ValueError("blocks must satisfy 0 < small < large")

    poses = cfg["poses"]
    _check_pose_list("pick", poses["pick"])
    _check_pose_list("place_left", poses["place_left"])
    poses["place_right"] = [mirror_pose(pose) for pose in poses["place_left"]]
    return cfg


def sim_stop_position(cfg, block_size):
    """Gripper joint position at which a block of this size would stop the fingers.

    Mock hardware has no physics, so simulation stands in for a real block.
    Assumed model: finger gap = gap_at_zero + 2 * joint_position, with gap_at_zero
    chosen so that gripper.size_threshold sits exactly halfway between the small
    and large block. This is a calibration assumption, not a measurement of the
    real gripper.
    """
    blocks = cfg["blocks"]
    threshold = cfg["gripper"]["size_threshold"]
    gap_at_zero = (blocks["small"] + blocks["large"]) / 2.0 - 2.0 * threshold
    return (block_size - gap_at_zero) / 2.0
