from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments for static TF
    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame', default_value='base_link',
        description='Parent frame for oakd_link.')
    child_frame_arg = DeclareLaunchArgument(
        'child_frame', default_value='oakd_link',
        description='Child frame name for the OAK-D link.')

    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Static TF x [m]')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Static TF y [m]')
    z_arg = DeclareLaunchArgument('z', default_value='0.0', description='Static TF z [m]')
    roll_arg = DeclareLaunchArgument('roll', default_value='0.0', description='Static TF roll [rad]')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.7864', description='Static TF pitch [rad]')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Static TF yaw [rad]')

    parent_frame = LaunchConfiguration('parent_frame')
    child_frame = LaunchConfiguration('child_frame')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    depth_node = Node(
        package='oak_d_lite',
        executable='depth_map',
        name='oakd_depth_map',
        output='screen'
    )

    # ROS 2 static_transform_publisher expects: x y z roll pitch yaw parent child
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oakd_static_tf',
        arguments=[
            x, y, z, roll, pitch, yaw,
            parent_frame, child_frame
        ],
        output='screen'
    )

    return LaunchDescription([
        parent_frame_arg,
        child_frame_arg,
        x_arg, y_arg, z_arg,
        roll_arg, pitch_arg, yaw_arg,
        depth_node,
        static_tf,
    ])
