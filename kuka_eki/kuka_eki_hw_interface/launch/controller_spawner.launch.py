import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument(
        "robot_description_package",
        default_value="kuka_kr3_support",
        description="Robot description package.",
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "robot_description_file",
        default_value="kr3r540.xacro",
        description="Robot description file located in <robot_description_package>/urdf/ .",
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description="Start robot with fake hardware mirroring command to its states.",
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "robot_ip",
        default_value="10.181.116.41",
        description="IP address by which the robot can be reached."
    ))

    declared_arguments.append(DeclareLaunchArgument(
        "robot_port",
        default_value="54600",
        description="Port by which the robot can be reached."
    ))

    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(robot_description_package), "urdf", robot_description_file]),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "robot_port:=",
            robot_port,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controllers_file = os.path.join(
        get_package_share_directory("kuka_resources"),
        'config',
        'kuka_6dof_controllers.yaml'
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file
        ],
        output='screen'
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    initial_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["position_trajectory_controller", "-c", "/controller_manager"],
    )
    return LaunchDescription(declared_arguments + [
        controller_manager, robot_state_publisher, joint_state_broadcaster_spawner,
        initial_joint_controller_spawner
    ])
