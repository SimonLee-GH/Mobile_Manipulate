"""Launch MoveIt2 move_group action server and the required bridges between Ignition and ROS 2"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from sdformat_tools.urdf_generator import UrdfGenerator


def generate_launch_description():
    pkg_universal_robot_ign = get_package_share_directory('universal_robot_ign')
    # Launch Arguments
    robot_sdf_path=os.path.join(pkg_universal_robot_ign, 'resource', 'models', 'ur5', 'model.sdf')
    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_file(robot_sdf_path)
    urdf_generator.remove_joint('world_ur5_joint')
    robot_urdf_xml = urdf_generator.to_string()
    rviz2_config = os.path.join(pkg_universal_robot_ign,"launch", "test.rviz")

    # Robot state publisher
    robot_state_publisher = Node(package="robot_state_publisher",
             executable="robot_state_publisher",
             name="robot_state_publisher",
             parameters=[{'robot_description': robot_urdf_xml}],
             output="screen")

    #static Robot state publisher
    static_transform_publisher_1=Node(package="tf2_ros",
             executable="static_transform_publisher",
             name="static_transform_publisher",
             output="screen",
             arguments=["0.0", "0.0", "1.4",
                        "0.0", "0.0", "0.0",
                        "world", "base_link"])
    
    static_transform_publisher_2=Node(package="tf2_ros",
             executable="static_transform_publisher",
             name="static_transform_publisher",
             output="screen",
             arguments=["0.0", "0.0", "0.089159",
                        "0.0", "0.0", "0.0",
                        "base_link", "shoulder_link"])
    
    static_transform_publisher_3=Node(package="tf2_ros",
             executable="static_transform_publisher",
             name="static_transform_publisher",
             output="screen",
             arguments=["0.0", "0.13585", "0.0",
                        "0.0", "1.5708", "0.0",
                        "shoulder_link", "upper_arm_link"])
    
    static_transform_publisher_4=Node(package="tf2_ros",
             executable="static_transform_publisher",
             name="static_transform_publisher",
             output="screen",
             arguments=["0.0", "-0.1197", "0.425",
                        "0.0", "0.0", "0.0",
                        "upper_arm_link", "forearm_link"])
    
    static_transform_publisher_5=Node(package="tf2_ros",
             executable="static_transform_publisher",
             name="static_transform_publisher",
             output="screen",
             arguments=["0.0", "0.0", "0.39225",
                        "0.0", "1.5708", "0.0",
                        "forearm_link", "wrist_1_link"])
    
    static_transform_publisher_6=Node(package="tf2_ros",
             executable="static_transform_publisher",
             name="static_transform_publisher",
             output="screen",
             arguments=["0.0", "0.093", "0.0",
                        "0.0", "0.0", "0.0",
                        "wrist_1_link", "wrist_2_link"])
    
    static_transform_publisher_7=Node(package="tf2_ros",
             executable="static_transform_publisher",
             name="static_transform_publisher",
             output="screen",
             arguments=["0.0", "0.0", "0.09465" ,
                        "0.0", "0.0", "0.0",
                        "wrist_2_link", "wrist_3_link"])
    
    # MoveIt2 move_group action server
    move_group=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_universal_robot_ign,"launch", "ur5_move_group_server.launch.py")
            ),
            # Simulation time does not function properly (as of Nov 2020), see https://github.com/AndrejOrsula/ign_moveit2/issues/4
        )
    # RViz2
    rviz2=Node(package="rviz2",
             executable="rviz2",
             name="rviz2",
             output="log",
             arguments=["--display-config", rviz2_config])   

    return LaunchDescription([
        robot_state_publisher,
        static_transform_publisher_1,
        move_group,
        rviz2,
        static_transform_publisher_2,
        static_transform_publisher_3,
        static_transform_publisher_4,
        static_transform_publisher_5,
        static_transform_publisher_6,
        static_transform_publisher_7,
    ])
