from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

from moveit_configs_utils import MoveItConfigsBuilder

from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    robot_description_path = get_package_share_directory('fr3_description')
    robot_bringup_path = get_package_share_directory('fr3_bringup')
    robot_moveit2_config_path = get_package_share_directory('fr3_weld_moveit2_config')

    urdf_path = os.path.join(robot_description_path, 'urdf', 'fr3.urdf.xacro')
    # 支持通过启动参数覆盖 RViz 配置
    rviz_config_default = os.path.join(robot_description_path, 'rviz', 'urdf_config.rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    srdf_path = os.path.join(robot_moveit2_config_path, 'config', 'fr3.srdf')
    controller_config_path = os.path.join(robot_moveit2_config_path, 'config', 'ros2_controllers.yaml')

    # XML格式的机器人描述文件
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    
    # C++控制节点
    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name="fr3_weld_moveit2_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .to_moveit_configs()
    )

    # 启动节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description},
                    {"publish_frequency": 30.0},],
    )

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_default,
        description='Path to RViz config file'
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_config_path],
        output='screen'
    )

    joint_state_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller"],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    fr3_executing_node = Node(
        package="fr3_weld_executing",
        executable="fr3_weld_executing",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # 延迟启动 fr3_executing，确保 move_group 已完全初始化
    delayed_fr3_executing = TimerAction(
        period=5.0,
        actions=[fr3_executing_node]
    )

    return LaunchDescription([
        declare_rviz_arg,
        LogInfo(msg=["Using RViz config: ", rviz_config]),
        robot_state_publisher_node,
        rviz_node,
        controller_manager_node,
        joint_state_controller_spawner,
        arm_controller_spawner,
        move_group_node,
        delayed_fr3_executing,
    ])
