from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    dlo_yaml = PathJoinSubstitution([
        FindPackageShare('direct_lidar_odometry'),
        'config',
        'dlo.yaml'
    ])
    params_yaml = PathJoinSubstitution([
        FindPackageShare('direct_lidar_odometry'),
        'config',
        'params.yaml'
    ])
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare('direct_lidar_odometry'),
        'launch',
        'dlo.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='robot'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('pointcloud_topic', default_value='lidar'),
        DeclareLaunchArgument('imu_topic', default_value='imu'),

        Node(
            package='direct_lidar_odometry',
            executable='dlo_odom_node',
            name='dlo_odom',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[dlo_yaml, params_yaml],
            remappings=[
                ('pointcloud', LaunchConfiguration('pointcloud_topic')),
                ('imu', LaunchConfiguration('imu_topic')),
                ('odom', 'dlo/odom_node/odom'),
                ('pose', 'dlo/odom_node/pose'),
                ('kfs', 'dlo/odom_node/odom/keyframe'),
                ('keyframe', 'dlo/odom_node/pointcloud/keyframe'),
            ]
        ),

        Node(
            package='direct_lidar_odometry',
            executable='dlo_map_node',
            name='dlo_map',
            namespace=LaunchConfiguration('robot_namespace'),
            output='screen',
            parameters=[dlo_yaml],
            remappings=[
                ('keyframes', 'dlo/odom_node/pointcloud/keyframe'),
                ('map', 'dlo/map_node/map'),
            ]
        )
    ])
