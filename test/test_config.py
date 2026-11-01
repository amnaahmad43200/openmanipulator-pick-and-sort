"""Unit tests for the configuration loader (no ROS needed)."""

import os

import pytest
import yaml

from openmanipulatorlab.config import load_config, mirror_pose, sim_stop_position

CONFIG_PATH = os.path.join(
    os.path.dirname(__file__), "..", "config", "challenge.yaml"
)


@pytest.fixture
def cfg():
    return load_config(CONFIG_PATH)


def write_modified(tmp_path, modify):
    """Write a variant of the real config and return its path."""
    with open(CONFIG_PATH, encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    modify(raw)
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return str(path)


def test_all_arm_poses_have_four_joints(cfg):
    for name in ("pick", "place_left", "place_right"):
        assert all(len(pose) == 4 for pose in cfg["poses"][name])


def test_right_poses_mirror_left_poses(cfg):
    left = cfg["poses"]["place_left"]
    right = cfg["poses"]["place_right"]
    assert len(left) == len(right)
    for left_pose, right_pose in zip(left, right):
        assert right_pose[0] == -left_pose[0]
        assert right_pose[1:] == left_pose[1:]


def test_mirror_pose_does_not_modify_input():
    pose = [1.0, 2.0, 3.0, 4.0]
    assert mirror_pose(pose) == [-1.0, 2.0, 3.0, 4.0]
    assert pose == [1.0, 2.0, 3.0, 4.0]


def test_sim_gripper_classifies_both_block_sizes(cfg):
    threshold = cfg["gripper"]["size_threshold"]
    small = sim_stop_position(cfg, cfg["blocks"]["small"])
    large = sim_stop_position(cfg, cfg["blocks"]["large"])
    assert small < threshold < large  # small closes past it, large stops short
    assert cfg["gripper"]["close_limit"] < small
    assert large < cfg["gripper"]["open_limit"]


def test_pose_with_wrong_joint_count_is_rejected(tmp_path):
    path = write_modified(tmp_path, lambda raw: raw["poses"]["pick"][0].pop())
    with pytest.raises(ValueError, match="expected 4"):
        load_config(path)


def test_threshold_outside_gripper_range_is_rejected(tmp_path):
    def modify(raw):
        raw["gripper"]["size_threshold"] = 0.5

    with pytest.raises(ValueError, match="size_threshold"):
        load_config(write_modified(tmp_path, modify))


def test_block_sizes_must_be_ordered(tmp_path):
    def modify(raw):
        raw["blocks"]["small"], raw["blocks"]["large"] = (
            raw["blocks"]["large"],
            raw["blocks"]["small"],
        )

    with pytest.raises(ValueError, match="small < large"):
        load_config(write_modified(tmp_path, modify))
