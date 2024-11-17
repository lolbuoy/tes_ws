import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            yaml_content = yaml.safe_load(file)
            if yaml_content is None:
                print(f"Warning: Empty YAML file: {absolute_file_path}")
            return yaml_content
    except EnvironmentError as e:
        print(f"Error loading YAML file {absolute_file_path}: {e}")
        return None

def generate_launch_description():
    # Get package path and verify SRDF
    pkg_path = get_package_share_directory("sapien_arm_moveit_config")
    robot_pkg_path = get_package_share_directory("sapien_arm")
    srdf_file = os.path.join(pkg_path, "config", "onshape.srdf")
    if not os.path.exists(srdf_file):
        raise RuntimeError(f"SRDF file not found at {srdf_file}")
    urdf_file = os.path.join(robot_pkg_path, "description", "onshape.urdf")  # Adjust path as needed
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="urdf")]),
            " ",
            urdf_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    # Command-line arguments
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )
    
    launch_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "true",
        "gripper": "link",
        "dof": "6",
    }
    print("lolol")
    # Build MoveIt Configuration
    joint_limits_yaml = load_yaml(
        "sapien_arm_moveit_config", "config/joint_limits.yaml"
    )
    print("ll")
    package_path = get_package_share_directory("sapien_arm_moveit_config")
    joint_limits_yaml = os.path.join(package_path, "config/joint_limits.yaml")
    print(joint_limits_yaml)
    moveit_config = (
            MoveItConfigsBuilder("onshape", package_name="sapien_arm_moveit_config")
            .robot_description(mappings=launch_arguments)
            .joint_limits(joint_limits_yaml)
            .robot_description_semantic(file_path="config/onshape.srdf")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")        
            .planning_scene_monitor(
                publish_robot_description=True, 
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
                publish_geometry_updates=True,
                publish_state_updates=True,
            )
            .planning_pipelines(
                pipelines=["ompl"],
            )
            .to_moveit_configs()
        )

    # Load configurations

    kinematics_yaml = load_yaml(
        "sapien_arm_moveit_config", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    moveit_simple_controllers_yaml = load_yaml(
        "sapien_arm_moveit_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "servo"],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,  # Direct reference, not wrapped
            {"use_sim_time": True}
        ],
    )
    planning_config = {
    "planning_plugin": "ompl_interface/OMPLPlanner",
    "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization 
                            default_planner_request_adapters/ResolveConstraintFrames
                            default_planner_request_adapters/FixWorkspaceBounds 
                            default_planner_request_adapters/FixStartStateBounds 
                            default_planner_request_adapters/FixStartStateCollision 
                            default_planner_request_adapters/FixStartStatePathConstraints""",
    "start_state_max_bounds_error": 0.1,
    "default_planner_config": "RRTConnectkConfigDefault",
    "maximum_waypoint_distance": 0.01,
}  
   
    move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                moveit_controllers,
                {"use_sim_time": True},
                {"publish_robot_description": True},
                {"publish_robot_description_semantic": True},
                {"allow_trajectory_execution": True},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                {"default_planning_pipeline": "ompl"},
                {"capabilities": """moveit_ros_move_group/MoveGroupCartesianPathService
                                moveit_ros_move_group/MoveGroupExecuteTrajectoryAction
                                moveit_ros_move_group/MoveGroupKinematicsService
                                moveit_ros_move_group/MoveGroupMoveAction
                                moveit_ros_move_group/MoveGroupPickPlaceAction
                                moveit_ros_move_group/MoveGroupPlanService
                                moveit_ros_move_group/MoveGroupQueryPlannersService
                                moveit_ros_move_group/MoveGroupStateValidationService
                                moveit_ros_move_group/MoveGroupGetPlanningSceneService
                                moveit_ros_move_group/ClearOctomapService"""},
                {"monitor_dynamics": False},
                {"cartesian_limits": {
                    "max_translation_speed": 1.0,
                    "max_translation_acceleration": 0.5,
                    "max_rotation_speed": 1.0,
                    "max_rotation_acceleration": 0.5
                }},
                {"trajectory_execution": {
                    "moveit_manage_controllers": True,
                    "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                    "trajectory_execution.allowed_goal_duration_margin": 0.5,
                    "trajectory_execution.allowed_start_tolerance": 0.01
                }}
            ],
            #arguments=['--ros-args', '--log-level', 'debug'],
        )

    # RViz
    rviz_base = os.path.join(pkg_path, "launch")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,  # Direct reference, not wrapped
            moveit_config.planning_pipelines,
            robot_description_kinematics,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("sapien_arm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    onshape_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

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
        condition=IfCondition(db_config),
    )

    # Debug prints
    print("\n============ Debug Information ============")
    print(f"Package path: {pkg_path}")
    print(f"SRDF file: {srdf_file}")
    print(f"Controllers path: {ros2_controllers_path}")
    print("\nPlanning Configuration:")
    print(planning_config)
    print("\nRobot Description Semantic:")
    print(moveit_config.robot_description_semantic)
    print("==========================================\n")

    return LaunchDescription(
        [
            db_arg,
            
            # Core components that must start first
            static_tf,
            robot_state_publisher,
            
            # ROS2 Control components with delays
            TimerAction(
                period=1.0,
                actions=[ros2_control_node]
            ),
            TimerAction(
                period=2.0,
                actions=[joint_state_broadcaster_spawner]
            ),
            TimerAction(
                period=3.0,
                actions=[onshape_controller_spawner]
            ),
            
            # MoveIt and visualization components
            TimerAction(
                period=4.0,
                actions=[move_group_node]
            ),
            TimerAction(
                period=5.0,
                actions=[rviz_node]
            ),
            
            # Optional components
            mongodb_server_node,
        ]
    )