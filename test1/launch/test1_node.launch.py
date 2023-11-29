import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetLaunchConfiguration
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = os.path.join(
      get_package_share_directory('test1'),
      'rviz',
      'default.rviz'
      )
    polygon_parameter_name = os.path.join(
      get_package_share_directory('test1'),
        'parameters',
        'example_polygon_epfl_simple.yaml'
      )
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='log',
            arguments=['-d', rviz_config]
        ),
        DeclareLaunchArgument(
            'parameter_file',
            default_value=os.path.join(
            get_package_share_directory('test1'),
                'parameters',
                'test1_node.yaml'
            ),
            description=''
        ),
        DeclareLaunchArgument(
            'parameter_file2',
            default_value=os.path.join(
            get_package_share_directory('test1'),
                'parameters',
                'example_polygon_epfl_simple.yaml'
            ),
            description=''
        ),
        Node(
            package='test1',
            executable='test1_node',
            name='test1_node',
            output='screen',
            # parameters=[
            # coverage_planner_parameter_name
            # # polygon_parameter_name
            # ]
            parameters=[LaunchConfiguration('parameter_file'),
            LaunchConfiguration('parameter_file2')]
        ),
        Node(
            package='test1',
            executable='test2_node',
            name='test2_node',
            output='screen',
            # parameters=[
            # coverage_planner_parameter_name
            # # polygon_parameter_name
            # ]
            parameters=[LaunchConfiguration('parameter_file')]
        ),
    ])

