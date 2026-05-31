from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg = FindPackageShare("grasp_bringup")

    # --- Robot description ---
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg, "urdf", "ur5_gz.urdf.xacro"]),
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    world_path = PathJoinSubstitution([pkg, "worlds", "grasp_world.sdf"])

    # Environment fixes for this machine:
    #   GZ_IP                   — gz-transport multicast doesn't bind loopback by default
    #   GZ_SIM_SYSTEM_PLUGIN_PATH — gz_ros2_control lives in /opt/ros/jazzy/lib, not Gazebo's path
    #   MESA_LOADER_DRIVER_OVERRIDE — force Mesa to use the iris driver (Intel Iris Xe)
    gz_env = {
        "GZ_IP": "127.0.0.1",
        "GZ_SIM_SYSTEM_PLUGIN_PATH": "/opt/ros/jazzy/lib",
        "MESA_LOADER_DRIVER_OVERRIDE": "iris",
    }

    # --- 1. Gazebo physics server (no GUI — OGRE2 renderer crashes on this hw) ---
    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", world_path],
        output="screen",
        additional_env=gz_env,
    )

    # --- 2. Robot State Publisher ---
    # Publishes /robot_description and TF frames for every link.
    # use_sim_time: nodes use /clock from Gazebo instead of wall clock.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # --- 3. Spawn UR5 into Gazebo ---
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "ur5", "-topic", "robot_description", "-world", "grasp_world"],
        output="screen",
        additional_env=gz_env,
    )

    # --- 4. Clock bridge: Gazebo sim time -> /clock topic ---
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
        additional_env=gz_env,
    )

    # --- 5. MoveIt2 configuration ---
    # MoveItConfigsBuilder reads from ur_moveit_config: SRDF, OMPL config,
    # kinematics (KDL IK solver), joint limits, planning scene monitor config.
    # We pass ur5 as the SRDF name substitution so it knows the planning group.
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur5"})
        .to_moveit_configs()
    )

    # Controller config override: the default ur_moveit_config lists
    # scaled_joint_trajectory_controller as default, which is UR hardware-only.
    # In simulation we only have joint_trajectory_controller, so we set it as default.
    moveit_controllers = {
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 1.2,
            "allowed_goal_duration_margin": 0.5,
            "allowed_start_tolerance": 0.01,
            "execution_duration_monitoring": False,
        },
        "moveit_simple_controller_manager": {
            "controller_names": ["joint_trajectory_controller"],
            "joint_trajectory_controller": {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            },
        },
    }

    # --- 6. move_group node ---
    # This is the MoveIt2 planning server. It:
    #   - listens for planning requests (from RViz or your action server)
    #   - runs OMPL to find collision-free paths
    #   - sends joint trajectories to joint_trajectory_controller for execution
    #   - maintains the planning scene (collision objects, robot state)
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_controllers,
            {"use_sim_time": True},
        ],
    )

    # --- 7. RViz with MoveIt2 MotionPlanning plugin ---
    # ur_moveit_config ships a pre-configured RViz layout with the
    # MotionPlanning panel, planning scene display, and trajectory visualiser.
    rviz_config = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"
    ])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # --- 8. ros2_control spawners ---
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

    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])
    delayed_controllers = TimerAction(period=6.0, actions=[joint_state_broadcaster_spawner])

    return LaunchDescription([
        gazebo_server,
        robot_state_publisher,
        gz_bridge,
        rviz,
        move_group,
        delayed_spawn,
        delayed_controllers,
        joint_trajectory_controller_spawner,
    ])
