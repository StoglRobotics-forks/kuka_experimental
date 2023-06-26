from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument(
        "robot_ip",
        default_value="10.181.116.41",
        description="IP address by which the robot can be reached."
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

    robot_ip = LaunchConfiguration("robot_ip")
    eki_io_port = LaunchConfiguration("eki_io_port")
    n_io = LaunchConfiguration("n_io")

    eki_io_node = Node(
        package="kuka_eki_io_interface",
        executable="eki_io_node",
        parameters=[
            {"robot_ip": robot_ip,
             "eki_io_port": eki_io_port,
             "n_io": n_io}
        ],
        emulate_tty=True,
        output='screen'
    )
    return LaunchDescription(declared_arguments + [
        eki_io_node
    ])
