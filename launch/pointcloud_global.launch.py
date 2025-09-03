import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for transform parameters
        DeclareLaunchArgument('x', default_value='0.0', description='X translation'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y translation'),
        DeclareLaunchArgument('z', default_value='0.0', description='Z translation'),
        DeclareLaunchArgument('roll', default_value='-1.5078', description='Roll rotation (radians)'),
        DeclareLaunchArgument('pitch', default_value='01.50780', description='Pitch rotation (radians)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw rotation (radians)'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Launch RViz'),
        
        # Robot state publisher for static transform from oakd_link->base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_oakd_base',
            arguments=[
                LaunchConfiguration('x'),
                LaunchConfiguration('y'), 
                LaunchConfiguration('z'),
                LaunchConfiguration('roll'),
                LaunchConfiguration('pitch'),
                LaunchConfiguration('yaw'),
                 
               
                'oakd_link',
                 'map'
            ],
            output='screen',
        ),
        # Pointcloud node (örnek, kendi node'unu buraya ekle)
        Node(
            package='oak_d_lite',
            executable='pointcloud',
            name='oak_pointcloud',
            output='screen',
            parameters=[{'frame_id': 'base_link'}],
        ),
        # İsteğe bağlı: RViz (varsayılan false)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
           # arguments=['-d', os.path.join(os.path.expanduser('~'), 'ros2_ws', 'rviz-nav2-conf.rviz')],
        ),
    ])
