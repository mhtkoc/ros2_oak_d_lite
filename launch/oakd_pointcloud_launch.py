from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


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
    roll_arg = DeclareLaunchArgument('roll', default_value='1.5708', description='Static TF roll [rad]')
    # Rotate -90 deg around Y so cloud looks parallel to ground by default
    pitch_arg = DeclareLaunchArgument('pitch', default_value='3.14159', description='Static TF pitch [rad]')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.57080', description='Static TF yaw [rad]')

    # Filter params
    z_min_arg = DeclareLaunchArgument('z_min', default_value='0.1', description='PassThrough min Z [m]')
    z_max_arg = DeclareLaunchArgument('z_max', default_value='3.0', description='PassThrough max Z [m]')
    mean_k_arg = DeclareLaunchArgument('mean_k', default_value='10', description='SOR mean_k')
    stddev_arg = DeclareLaunchArgument('stddev', default_value='0.5', description='SOR stddev threshold')

    # Simple URDF to visualize camera as a 5x5x2 box at oakd_link
    urdf = """
<robot name="oakd_visual">
  <link name="oakd_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="5 5 2"/>
      </geometry>
      <material name="camera_blue">
        <color rgba="0.0 0.4 1.0 0.5"/>
      </material>
    </visual>
  </link>
</robot>
"""

    parent_frame = LaunchConfiguration('parent_frame')
    child_frame = LaunchConfiguration('child_frame')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    z_min = LaunchConfiguration('z_min')
    z_max = LaunchConfiguration('z_max')
    mean_k = LaunchConfiguration('mean_k')
    stddev = LaunchConfiguration('stddev')

    depth_node = Node(
        package='oak_d_lite',
        executable='depth_map',
        name='oakd_depth_map',
        output='screen'
    )

    # Visualize oakd_link as a box in RViz via robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='oakd_state_publisher',
        parameters=[{'robot_description': urdf}],
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

    # Composable container with point cloud filters
    filter_container = ComposableNodeContainer(
        name='oakd_filter_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::PassThrough',
                name='passthrough_filter',
                parameters=[{
                    'filter_field_name': 'z',
                    'filter_limit_min': z_min,
                    'filter_limit_max': z_max,
                    'filter_limit_negative': False,
                }],
                remappings=[
                    ('input', '/camera/depth/points'),
                    ('output', '/camera/depth/points_filtered_z'),
                ],
            ),
            ComposableNode(
                package='pcl_ros',
                plugin='pcl_ros::StatisticalOutlierRemoval',
                name='statistical_outlier_filter',
                parameters=[{
                    'mean_k': mean_k,
                    'stddev': stddev,
                }],
                remappings=[
                    ('input', '/camera/depth/points_filtered_z'),
                    ('output', '/camera/depth/points_filtered'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        parent_frame_arg,
        child_frame_arg,
        x_arg, y_arg, z_arg,
        roll_arg, pitch_arg, yaw_arg,
        z_min_arg, z_max_arg, mean_k_arg, stddev_arg,
        depth_node,
        rsp,
        static_tf,
        filter_container,
    ])
