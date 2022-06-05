"""Launch MoveIt2 move_group action server and the required bridges between Ignition and ROS 2"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    pkg_universal_robot_ign = get_package_share_directory('universal_robot_ign')
    # Launch Arguments
    urdf = os.path.join(pkg_universal_robot_ign, "resource", "urdf", "ur10.urdf")
    rviz2_config = os.path.join(pkg_universal_robot_ign, "launch", "test.rviz")

    # Robot state publisher
    robot_state_publisher = Node(package="robot_state_publisher",
                                 executable="robot_state_publisher",
                                 name="robot_state_publisher",
                                 output="screen",
                                 arguments=[urdf])
    # static Robot state publisher
    static_transform_publisher = Node(package="tf2_ros",
                                      executable="static_transform_publisher",
                                      name="static_transform_publisher",
                                      output="screen",
                                      arguments=["0.0", "0.0", "1.4",
                                                 "0.0", "0.0", "0.0",
                                                 "world", "base_link"])
    # MoveIt2 move_group action server
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_universal_robot_ign, "launch", "ur10_move_group_server.launch.py")
        ),
        # Simulation time does not function properly (as of Nov 2020), see https://github.com/AndrejOrsula/ign_moveit2/issues/4
    )

    # URDF
    robot_urdf_config = load_file("universal_robot_ign","resource/urdf/ur10.urdf")
    robot_description = {"robot_description": robot_urdf_config}
    ## SRDF
    robot_srdf = load_file("universal_robot_ign","resource/ur10_moveit_config/ur10.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_srdf}
    # Kinematics
    kinematics = load_yaml("universal_robot_ign","resource/ur10_moveit_config/kinematics.yaml")
    # Joint limits
    joint_limits_yaml = load_yaml("universal_robot_ign", "resource/ur10_moveit_config/joint_limits.yaml")
    joint_limits = {"robot_description_planning": joint_limits_yaml}
    # Planning
    ompl_yaml = load_yaml("universal_robot_ign","resource/ur10_moveit_config/ompl_planning.yaml")
    # planning = {"move_group": {
    #     "planning_plugin": "ompl_interface/OMPLPlanner",
    #     "request_adapters": """default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #     "start_state_max_bounds_error": 0.1}}

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(ompl_yaml)


    # Trajectory Execution
    trajectory_execution = {"allow_trajectory_execution": False,
                            "moveit_manage_controllers": False}

    # Planning Scene
    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                                         "publish_geometry_updates": True,
                                         "publish_state_updates": True,
                                         "publish_transforms_updates": True}
    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # RViz2
    rviz2 = Node(package="rviz2",
                 executable="rviz2",
                 name="rviz2",
                 output="log",
                 arguments=["--display-config", rviz2_config],
                 parameters=[robot_description,
                             robot_description_semantic,
                             kinematics,
                             joint_limits,
                             ompl_planning_pipeline_config,
                             ompl_yaml,
                             moveit_controllers,
                             trajectory_execution,
                             planning_scene_monitor_parameters,
                             {"use_sim_time": False}])

    return LaunchDescription([
        robot_state_publisher, static_transform_publisher, move_group, rviz2
    ])


generate_launch_description()
