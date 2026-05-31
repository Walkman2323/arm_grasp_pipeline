from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("grasp_bringup")

    # --- Robot description (processed by xacro CLI at launch time) ---
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg, "urdf", "ur5_gz.urdf.xacro"]),
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    world_path = PathJoinSubstitution([pkg, "worlds", "grasp_world.sdf"])

    # --- 1. Gazebo server only (-s flag) ---
    # The GUI (OGRE2 renderer) crashes on Intel Iris Xe with the vendored
    # Gazebo Harmonic build. Running server-only bypasses the renderer entirely.
    # RViz2 below handles all visualization — it uses a different render stack.
    #
    # GZ_IP=127.0.0.1: gz-transport uses UDP multicast for process discovery.
    # Without this, it doesn't bind to the loopback interface and processes
    # on the same machine can't find each other.
    gz_env = {
        "GZ_IP": "127.0.0.1",
        "MESA_LOADER_DRIVER_OVERRIDE": "iris",
        # Gazebo's plugin loader only searches its own install paths by default.
        # gz_ros2_control lives in the ROS2 lib dir, so we add it explicitly.
        "GZ_SIM_SYSTEM_PLUGIN_PATH": "/opt/ros/jazzy/lib",
    }

    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", world_path],
        output="screen",
        additional_env=gz_env,
    )

    # --- 2. Robot State Publisher ---
    # Reads the URDF, publishes TF transforms for every link.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # --- 3. Spawn robot into Gazebo ---
    # -world: skip the world-name discovery step (requires working gz-transport
    #         multicast) and go directly to the known world name.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "ur5", "-topic", "robot_description", "-world", "grasp_world"],
        output="screen",
        additional_env=gz_env,
    )

    # --- 4. Bridges: Gazebo transport <-> ROS2 topics ---
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
        additional_env=gz_env,
    )

    # --- 5. Controller spawners ---
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

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

    # --- 6. RViz2 ---
    # Replaces the Gazebo GUI for visualization. Shows robot model from URDF,
    # TF tree, joint states, and later: point clouds and planned trajectories.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])
    delayed_controllers = TimerAction(period=6.0, actions=[joint_state_broadcaster_spawner])

    return LaunchDescription([
        gazebo_server,
        robot_state_publisher,
        gz_bridge,
        rviz,
        delayed_spawn,
        delayed_controllers,
        joint_trajectory_controller_spawner,
    ])
