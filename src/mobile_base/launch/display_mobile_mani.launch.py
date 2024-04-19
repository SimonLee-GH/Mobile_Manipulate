import launch
import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    NotSubstitution,
    AndSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

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
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="mobile_base"
    ).find("mobile_base")

    pkg_universal_robot_ign = launch_ros.substitutions.FindPackageShare(
        package="universal_robot_ign"
    ).find("universal_robot_ign")

    default_model_path = os.path.join(
        pkg_share, "urdf/mobile_manipulator/mobile_manipulator.urdf"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz/urdf_config.rviz")
  
    world_path = os.path.join(pkg_share, "world/empty.sdf")
    # world_path = "/home/simon/two_wheeled_robot_worlds/cafe.world"
    gz_models_path = os.path.join(pkg_share, "models")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_localization = LaunchConfiguration("use_localization")
    use_rviz = LaunchConfiguration("use_rviz")
    log_level = LaunchConfiguration("log_level")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    run_headless = LaunchConfiguration("run_headless")

    with open(default_model_path, 'r') as file:
        robot_description = file.read()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )



    rviz_node = Node(
        condition=IfCondition(AndSubstitution(NotSubstitution(run_headless), use_rviz)),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    # Localize using odometry and IMU data. 
    # It can be turned off because the navigation stack uses AMCL with lidar data for localization
    robot_localization_node = Node(
        condition=launch.conditions.IfCondition(use_localization),
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # gazebo have to be executed with shell=False, or test_launch won't terminate it
    #   see: https://github.com/ros2/launch/issues/545
    # This code is form taken ros_gz_sim and modified to work with shell=False
    #   see: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/launch/gz_sim.launch.py.in
    gz_env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  # TODO(CH3): To support pre-garden. Deprecated.
                      ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                                os.environ.get('LD_LIBRARY_PATH', default='')])}
    gazebo = [
        ExecuteProcess(
            condition=launch.conditions.IfCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, '-s', '--headless-rendering', world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        ),
        ExecuteProcess(
            condition=launch.conditions.UnlessCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        )
    ]

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "mobile_base",
            "-topic",
            "robot_description",
            "-z",
            "0.3",
            "-x",
            "-2.0",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/sky_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/robot_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            # Clock message is necessary for the diff_drive_controller to accept commands https://github.com/ros-controls/gz_ros2_control/issues/106
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        shell=False,
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        name="activate_ackermann_steering_controller",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "ackermann_steering_controller",
        ],
        shell=False,
        output="screen",
    )

    
    odom_pub_node = Node(
        package='mobile_base',  # 替换为包含odompub.py的包名
        executable='odompub.py',
        name='odom_pub_node',
        output='screen',
    )

    return launch.LaunchDescription(
        [
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=gz_models_path,
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_MODEL_PATH",
                value=gz_models_path,
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="True",
                description="Start RViz",
            ),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in headless mode and don't start RViz (overrides use_rviz)",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="False",
                description="Flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                name="use_localization",
                default_value="False",
                description="Use EKF to estimagte odom->base_link transform from IMU + wheel odometry",
            ),
            DeclareLaunchArgument(
                "gz_verbosity",
                default_value="3",
                description="Verbosity level for Ignition Gazebo (0~4).",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="",
                description="Extra args for Gazebo (ie. '-s' for running headless)",
            ),
            DeclareLaunchArgument(
                name="log_level",
                default_value="warn",
                description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
            ),
            bridge,
            robot_state_publisher_node,
            spawn_entity,
            robot_localization_node,
            rviz_node,
            odom_pub_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_joint_trajectory_controller],
                )
            ),
           
        ] + gazebo
    )
    # return launch.LaunchDescription([
    #         DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
    #                                 description='Flag to enable joint_state_publisher_gui'),
    #         DeclareLaunchArgument(
    #                 name="model",
    #                 default_value=default_model_path,
    #                 description="Absolute path to robot urdf file",
    #             ),
    #         DeclareLaunchArgument(
    #              name="rvizconfig",
    #              default_value=default_rviz_config_path,
    #             description="Absolute path to rviz config file",
    #          ),
    #         DeclareLaunchArgument(
    #              name="run_headless",
    #              default_value="False",
    #              description="Start GZ in hedless mode and don't start RViz (overrides use_rviz)",
    #          ),
    #         DeclareLaunchArgument(
    #              name="use_rviz",
    #              default_value="True",
    #              description="Start RViz",
    #          ),
    #         joint_state_publisher_node,
    #         joint_state_publisher_gui_node,
    #         robot_state_publisher_node,
    #         rviz_node
    #         ])