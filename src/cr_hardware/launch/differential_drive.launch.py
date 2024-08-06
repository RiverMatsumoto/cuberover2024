from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
import os

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "debug_diff_drive",
            default_value="false",
            description="Enable debug output and behavior for the diff drive controller.",
        )
    )

    diff_drive_yaml_file = os.path.abspath(os.path.join(get_package_share_directory('cr_hardware'), 
                                                        'config', 
                                                        'differential_drive.yaml'))
    with open(diff_drive_yaml_file  , 'r') as config_file:
        config_params = yaml.safe_load(config_file)
    diff_drive_ros_params = config_params["differential_drive_params"]["ros__parameters"]
    diff_drive_controller_ros_params = config_params["differential_drive_controller"]["ros__parameters"]
    debug_diff_drive = diff_drive_ros_params["debug_diff_drive"]
    serial_port = diff_drive_ros_params["serial_port"]
    baud_rate = diff_drive_ros_params["baud_rate"]
    wheel_radius = diff_drive_controller_ros_params["wheel_radius"]

    # Initialize diff drive plugin arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    # debug_diff_drive = LaunchConfiguration("debug_diff_drive")

    # get urdf with xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", PathJoinSubstitution([FindPackageShare("cr_hardware"), "urdf", "cuberover.urdf.xacro"]),
            " use_mock_hardware:=", use_mock_hardware,
            " debug:=", str(debug_diff_drive),
            " serial_port:=", serial_port,
            " baud_rate:=", str(baud_rate),
            " wheel_radius:=", str(wheel_radius)
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("cr_hardware"),
            "config",
            "differential_drive.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_demo_description"), "diffbot/rviz", "diffbot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["differential_drive_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
