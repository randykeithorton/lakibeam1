import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame_id_0 = LaunchConfiguration('frame_id_0')
    frame_id_1 = LaunchConfiguration('frame_id_1')
    output_topic0 = LaunchConfiguration('output_topic0')
    output_topic1 = LaunchConfiguration('output_topic1')
    inverted = LaunchConfiguration('inverted')
    hostip_0 = LaunchConfiguration('hostip_0')
    hostip_1 = LaunchConfiguration('hostip_1')
    port0 = LaunchConfiguration('port0')
    port1 = LaunchConfiguration('port1')
    angle_offset = LaunchConfiguration('angle_offset')
    scanfreq = LaunchConfiguration('scanfreq')
    filter = LaunchConfiguration('filter')
    laser_enable = LaunchConfiguration('laser_enable')
    scan_range_start = LaunchConfiguration('scan_range_start')
    scan_range_stop = LaunchConfiguration('scan_range_stop')
    sensorip = LaunchConfiguration('sensorip')


    

    declare_frame_id0_cmd = DeclareLaunchArgument(
    'frame_id_0',
    default_value='front_lidar',
    )
    declare_frame_id1_cmd = DeclareLaunchArgument(
    'frame_id_1',
    default_value='rear_lidar',
    )
    declare_output_topic0_cmd = DeclareLaunchArgument(
    'output_topic0',
    default_value='scan_0',
    )
    declare_output_topic1_cmd = DeclareLaunchArgument(
    'output_topic1',
    default_value='scan_1',
    )
    declare_inverted_cmd = DeclareLaunchArgument(
    'inverted',
    default_value='false',
    )
    declare_hostip0_cmd = DeclareLaunchArgument(
    'hostip_0',
    default_value='192.168.8.1',
    )
    declare_hostip1_cmd = DeclareLaunchArgument(
    'hostip_1',
    default_value='192.168.8.3',
    )
    declare_port0_cmd = DeclareLaunchArgument(
    'port0',
    default_value='"2368"',
    )
    declare_port1_cmd = DeclareLaunchArgument(
    'port1',
    default_value='"2369"',
    )
    declare_angle_offset_cmd = DeclareLaunchArgument(
    'angle_offset',
    default_value='0',
    )
    declare_filter_cmd = DeclareLaunchArgument(
    'filter',
    default_value='"3"',
    )
    declare_scanfreq_cmd = DeclareLaunchArgument(
    'scanfreq',
    default_value='"30"',
    )
    declare_laser_enable_cmd = DeclareLaunchArgument(
    'laser_enable',
    default_value='"true"',
    )
    declare_scan_range_start_cmd = DeclareLaunchArgument(
    'scan_range_start',
    default_value='"45"',
    )
    declare_scan_range_stop_cmd = DeclareLaunchArgument(
    'scan_range_stop',
    default_value='"315"',
    )
    declare_sensorip_cmd = DeclareLaunchArgument(
    'sensorip',
    default_value='192.168.198.2',
    )

    richbeam_lidar_node0 = Node(
        package='lakibeam1',
        name='richbeam_lidar_node0',
        executable='lakibeam1_scan_node',
        parameters=[{
            'frame_id':frame_id_0,
            'output_topic':output_topic0,
            'inverted':inverted,
            'hostip':hostip_0,
            'port':port0,
            'angle_offset':angle_offset
        }],
        output='screen'
    )
    richbeam_lidar_node1 = Node(
        package='lakibeam1',
        name='richbeam_lidar_node1',
        executable='lakibeam1_scan_node',
        parameters=[{
            'frame_id':frame_id_1,
            'output_topic':output_topic1,
            'inverted':inverted,
            'hostip':hostip_1,
            'port':port0,
            'angle_offset':angle_offset
        }],
        output='screen'
    )
    lakibeam1_pcd_dir = get_package_share_directory('lakibeam1')
    # rviz_config_dir = os.path.join(lakibeam1_pcd_dir,'rviz','lakibeam1_scan_dual.rviz')
    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # rviz_node = Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         name='rviz2',
    #         arguments=['-d', rviz_config_dir],
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         output='screen')
    ld = LaunchDescription()

    ld.add_action(declare_frame_id0_cmd)
    ld.add_action(declare_frame_id1_cmd)
    ld.add_action(declare_output_topic0_cmd)
    ld.add_action(declare_output_topic1_cmd)
    ld.add_action(declare_inverted_cmd)
    ld.add_action(declare_hostip0_cmd)
    ld.add_action(declare_hostip1_cmd)
    ld.add_action(declare_port0_cmd)
    ld.add_action(declare_port1_cmd)
    ld.add_action(declare_angle_offset_cmd)
    ld.add_action(declare_filter_cmd)
    ld.add_action(declare_scanfreq_cmd)
    ld.add_action(declare_laser_enable_cmd)
    ld.add_action(declare_scan_range_start_cmd)
    ld.add_action(declare_scan_range_stop_cmd)
    ld.add_action(declare_sensorip_cmd)
    ld.add_action(richbeam_lidar_node0)
    ld.add_action(richbeam_lidar_node1)
    # ld.add_action(rviz_node)
    return ld
