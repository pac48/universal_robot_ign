"""Launch move_group action server without controllers and trajectory execution
# refenerce:
# https://github.com/AndrejOrsula/panda_moveit2_config/blob/master/launch/move_group_action_server.launch.py
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)

    # URDF
    robot_urdf_config = load_file("universal_robot_ign","resource/urdf/ur10.urdf")
    robot_description = {"robot_description": robot_urdf_config}

    ## SRDF
    robot_srdf = load_file("universal_robot_ign","resource/ur10_moveit_config/ur10.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_srdf}
    # MoveIt Configuration

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "moveit_config_file",
    #         default_value="ur.srdf.xacro",
    #         description="MoveIt SRDF/XACRO description file with the robot.",
    #     )
    # )
    #
    # moveit_config_package = "/home/paul/admittance_ws/src/universal_robot_ign/resource/ur10_moveit_config/"
    # moveit_config_file = "ur10.srdf"
    # prefix=""
    #
    # robot_description_semantic_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution([moveit_config_package, moveit_config_file])
    #         ,
    #         " ",
    #         "name:=",
    #         # Also ur_type parameter could be used but then the planning group names in yaml
    #         # configs has to be updated!
    #         "ur",
    #         " ",
    #         "prefix:=",
    #         prefix,
    #         " ",
    #     ]
    # )
    # robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}


    # robot_description_semantic = {"robot_description_semantic": ''}


    # Kinematics
    kinematics = load_yaml("universal_robot_ign","resource/ur10_moveit_config/kinematics.yaml")

    # Joint limits
    joint_limits_yaml = load_yaml("universal_robot_ign", "resource/ur10_moveit_config/joint_limits.yaml")
    joint_limits = {"robot_description_planning": joint_limits_yaml}

    # Planning
    ompl_yaml = load_yaml("universal_robot_ign","resource/ur10_moveit_config/ompl_planning.yaml")

    planning = {"move_group": {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": """default_planner_request_adapters/AddTimeParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        "start_state_max_bounds_error": 0.1}}
    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Trajectory Execution
    trajectory_execution = {"allow_trajectory_execution": False,
                            "moveit_manage_controllers": False}

    # Planning Scene
    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                                         "publish_geometry_updates": True,
                                         "publish_state_updates": True,
                                         "publish_transforms_updates": True}

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=use_sim_time,
            description="If true, use simulated clock"),

        # Start move_group action server
        Node(package="moveit_ros_move_group",
             executable="move_group",
             name="move_group_2",
             output="screen",
             parameters=[robot_description,
                         robot_description_semantic,
                         kinematics,
                         joint_limits,
                         planning,
                         moveit_controllers,
                         ompl_yaml,
                         trajectory_execution,
                         planning_scene_monitor_parameters,
                         {"use_sim_time": use_sim_time}]),
    ])


    # Start move_group action server
    # move_group_node = Node(package="moveit_ros_move_group",
    #                        executable="move_group",
    #                        name="move_group",
    #                        output="screen",
    #                        parameters=[robot_description,
    #                                    robot_description_semantic,
    #                                    kinematics,
    #                                    joint_limits,
    #                                    planning,
    #                                    moveit_controllers,
    #                                    ompl_yaml,
    #                                    trajectory_execution,
    #                                    planning_scene_monitor_parameters,
    #                                    {"use_sim_time": use_sim_time}])
    #
    # return LaunchDescription([DeclareLaunchArgument(
    #     "use_sim_time",
    #     default_value=use_sim_time,
    #     description="If true, use simulated clock"), move_group_node
    # ])
