import os
import yaml
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro
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
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="kuka_6dof_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_package",
            description="Description package with robot URDF/xacro files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_macro_file",
            default_value="kr3r540_macro.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "semantic_description_package",
            default_value="kuka_common_moveit_config",
            description="Semantic robot description file located in <robot_description_package>/config/ .",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "semantic_description_file",
            default_value="common_kuka.srdf.xacro",
            description="Semantic robot description file located in <robot_description_package>/config/ .",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="kuka",
            description="NOTE:robot name and robot description macro name are same",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="172.0.0.1",
            description="IP address by which the robot can be reached."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port",
            default_value="49152",
            description="Port by which the robot can be reached."
        )
    )

    # initialize arguments
    controllers_file = LaunchConfiguration("controllers_file")
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_macro_file = LaunchConfiguration("robot_description_macro_file")
    robot_name = LaunchConfiguration("robot_name")
    semantic_description_package = LaunchConfiguration("semantic_description_package")
    semantic_description_file = LaunchConfiguration("semantic_description_file")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    prefix = LaunchConfiguration("prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kuka_ros2_control_support"), "urdf", "common_kuka.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "controllers_file:=",
            controllers_file,
            " ",
            "robot_description_package:=",
            robot_description_package,
            " ",
            "robot_description_macro_file:=",
            robot_description_macro_file,
            " ",
            "robot_name:=",
            robot_name,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "robot_port:=",
            robot_port,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(semantic_description_package), "config", semantic_description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "robot_name:=",
            robot_name,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    kinematics_yaml = load_yaml(
        "kuka_common_moveit_config", "config/kinematics.yaml"
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "kuka_common_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        "kuka_common_moveit_config", "config/kuka_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory("kuka_common_moveit_config"), "config")
    rviz_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    # ros2_control using FakeSystem as hardware
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("kuka_resources"), "config", controllers_file]
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Load controllers
    load_controllers = []
    for controller in ["position_trajectory_controller", "joint_state_broadcaster"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # Warehouse mongodb server
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config)
    )

    return LaunchDescription(
        declared_arguments + [
            db_arg,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
            mongodb_server_node,
        ]
        + load_controllers
    )
