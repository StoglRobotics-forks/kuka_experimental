from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.181.116.41",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(DeclareLaunchArgument(
        "eki_robot_port",
        default_value="54600",
        description="Port by which the robot can be reached."
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "eki_io_port",
        default_value="54601",
        description="Port by which the robot can be reached."
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "n_io",
        default_value="2",
        description="Port by which the robot can be reached."
    ))

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    eki_robot_port = LaunchConfiguration("eki_robot_port")
    eki_io_port = LaunchConfiguration("eki_io_port")
    n_io = LaunchConfiguration("n_io")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("kuka_kr3_cell_description"), "urdf", "robot.urdf.xacro"]),
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "eki_robot_port:=",
            eki_robot_port,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kuka_kr3_cell_description"), "config", "robot.srdf"]
            ),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('kuka_common_moveit_config'), 'launch', 'demo.launch.py'])),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "eki_robot_port": eki_robot_port,
            "robot_description_package": 'kuka_kr3_cell_description'
        }.items(),
    )

    moveit_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('moveit_wrapper'), 'launch', 'moveit_wrapper.launch.py'])),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "eki_robot_port": eki_robot_port,
            "robot_description_package": 'kuka_kr3_cell_description'
        }.items(),
    )

    kinematics_yaml = load_yaml("kuka_common_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}
    servo_yaml = load_yaml("kuka_kr3_cell_description", "config/kuka_servo_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_wrapper",
        executable="servo_server",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [moveit_launch, moveit_wrapper_launch, servo_node])
