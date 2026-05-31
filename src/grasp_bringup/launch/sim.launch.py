from pathlib import Path
import xacro
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory("grasp_bringup")

    # --- 1. Parse URDF from xacro ---
    # xacro processes the .xacro file and produces a plain URDF string.
    # We pass it as a parameter to robot_state_publisher below.
    urdf_path = Path(pkg) / "urdf" / "ur5_gz.urdf.xacro"
    robot_description = xacro.process_file(str(urdf_path)).toxml()

    world_path = Path(pkg) / "worlds" / "grasp_world.sdf"

    # --- 2. Gazebo Harmonic ---
    # gz sim starts the physics simulation. -r means "run immediately".
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", str(world_path)],
        output="screen",
    )

    # --- 3. Robot State Publisher ---
    # Reads the URDF and publishes TF transforms for every link.
    # This is how RViz and other nodes know where each part of the robot is.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # --- 4. Spawn the robot into Gazebo ---
    # ros_gz_sim reads the robot_description topic and spawns the model.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ur5",
            "-topic", "robot_description",
        ],
        output="screen",
    )

    # --- 5. Bridge: Gazebo <-> ROS2 ---
    # Gazebo uses its own transport layer. This bridge relays specific topics
    # in both directions so ROS2 nodes can read sensors and send commands.
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    # --- 6. ros2_control spawners ---
    # These activate the controllers we defined in controllers.yaml.
    # joint_state_broadcaster must start first — it publishes /joint_states,
    # which the trajectory controller and MoveIt2 both depend on.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Wait for joint_state_broadcaster to be active before spawning the
    # trajectory controller — controllers have a strict startup dependency.
    joint_trajectory_controller_spawner = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_trajectory_controller"],
                    output="screen",
                )
            ],
        )
    )

    # Give Gazebo and robot_state_publisher a moment to start before
    # spawning the robot and activating controllers.
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        gz_bridge,
        delayed_spawn,
        delayed_controllers,
        joint_trajectory_controller_spawner,
    ])
