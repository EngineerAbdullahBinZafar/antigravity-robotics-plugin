import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    """
    Launch Gazebo Harmonic Zero-G Simulation and Antigravity Lifecycle Controller.
    """
    world_file = os.path.join(
        os.path.dirname(__file__), "..", "worlds", "zero_g_orbit.sdf"
    )

    # Gazebo Sim Process
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

    # ROS-GZ Bridge for /cmd_vel, /scan, and /imu
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        output="screen"
    )

    # Antigravity Lifecycle Controller Node
    lifecycle_controller = LifecycleNode(
        package="antigravity_core",
        executable="antigravity_controller_node",
        name="antigravity_lifecycle_controller",
        namespace="",
        output="screen",
        parameters=[{
            "max_thrust_n": 150.0,
            "damping_factor": 0.92,
            "watchdog_timeout_ms": 250,
        }]
    )

    return LaunchDescription([
        gz_sim,
        ros_gz_bridge,
        lifecycle_controller,
    ])
