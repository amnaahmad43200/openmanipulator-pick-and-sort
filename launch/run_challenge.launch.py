import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim = LaunchConfiguration("sim")

    bringup_launch = os.path.join(
        get_package_share_directory("open_manipulator_bringup"), "launch", "open_manipulator_x.launch.py"
    )
    moveit_launch = os.path.join(
        get_package_share_directory("open_manipulator_moveit_config"), "launch", "open_manipulator_x_moveit.launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="true", description="Use mock hardware (true) or real robot (false)"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={"use_mock_hardware": sim, "start_rviz": "false"}.items(),
        ),
        # Note, use_sim must be false because we aren't using gazebo (even if we aren't using a real arm)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={"use_sim": 'false', "start_rviz": "true"}.items(),
        ),
    ])
