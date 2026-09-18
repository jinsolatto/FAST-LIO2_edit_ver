import os
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node


def _resolve_config_file(config_path_value, config_file_value):
    config_file = Path(config_file_value)
    if config_file.is_absolute():
        return config_file
    return Path(config_path_value) / config_file


def _load_ros_params(config_file):
    with open(config_file, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream) or {}


def _local_ba_weight_profile(name):
    """Return an explicit Step-2A IMU weighting experiment profile.

    ``custom`` returns no overrides, preserving the supplied YAML.  A/B/C
    deliberately disable both feedback paths: these are BA-only experiments.
    """
    profiles = {'A': (1.0, 1.0, 1.0), 'B': (3.0, 1.0, 2.0), 'C': (6.0, 1.0, 3.0)}
    normalized = name.strip().upper()
    if normalized in ('', 'CUSTOM'):
        return {}
    if normalized not in profiles:
        raise RuntimeError('local_ba_weight_set must be custom, A (1:1:1), B (3:1:2), or C (6:1:3); got {!r}'.format(name))
    rot, vel, pos = profiles[normalized]
    return {
        'mapping.local_ba_enable': True,
        'mapping.local_ba_imu_enable': True,
        'mapping.local_ba_imu_rot_weight': rot,
        'mapping.local_ba_imu_vel_weight': vel,
        'mapping.local_ba_imu_pos_weight': pos,
        'mapping.local_ba_map_feedback_enable': False,
        'mapping.local_ba_ekf_pose_feedback_enable': normalized == 'C',
        'mapping.local_ba_ekf_feedback_max_translation': 0.05,
        'mapping.local_ba_ekf_feedback_max_rotation_deg': 0.5,
        'mapping.local_ba_ekf_feedback_max_velocity': 0.20,
    }


def _find_velodyne_calibration_file(package_path, scan_line):
    candidates = []
    if scan_line == 16:
        candidates = ['VLP16db.yaml', 'VLP16.yaml']
    elif scan_line == 32:
        candidates = ['VLP32Cdb.yaml', '32db.yaml', 'HDL32Edb.yaml']
    elif scan_line == 64:
        candidates = ['64e_s2.1-sztaki.yaml', 'HDL64E_S3-xiesc.yaml']

    for candidate in candidates:
        calibration = Path(package_path) / 'params' / candidate
        if calibration.exists():
            return str(calibration)
    return ''


def _launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    config_path = LaunchConfiguration('config_path').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    rviz_cfg = LaunchConfiguration('rviz_cfg').perform(context)
    rviz_use = LaunchConfiguration('rviz').perform(context)
    auto_convert = LaunchConfiguration('velodyne_auto_convert').perform(context).lower() == 'true'
    velodyne_model = LaunchConfiguration('velodyne_model').perform(context)
    velodyne_calibration = LaunchConfiguration('velodyne_calibration').perform(context)
    velodyne_points_topic = LaunchConfiguration('velodyne_points_topic').perform(context)
    camera_republish = LaunchConfiguration('camera_republish').perform(context).lower() == 'true'
    camera_compressed_topic = LaunchConfiguration('camera_compressed_topic').perform(context)
    camera_raw_topic = LaunchConfiguration('camera_raw_topic').perform(context)
    local_ba_weight_set = LaunchConfiguration('local_ba_weight_set').perform(context)

    resolved_config = _resolve_config_file(config_path, config_file)
    params = _load_ros_params(resolved_config)
    ros_params = params.get('/**', {}).get('ros__parameters', {})
    common_params = ros_params.get('common', {})
    preprocess_params = ros_params.get('preprocess', {})

    lidar_type = preprocess_params.get('lidar_type')
    lidar_topic = common_params.get('lid_topic', '')
    scan_line = int(preprocess_params.get('scan_line', 0))
    use_velodyne_packets = (
        auto_convert and lidar_type == 2 and lidar_topic == '/velodyne_packets'
    )

    fast_lio_parameters = [str(resolved_config), {'use_sim_time': use_sim_time.lower() == 'true'}]
    weight_profile = _local_ba_weight_profile(local_ba_weight_set)
    if weight_profile:
        fast_lio_parameters.append(weight_profile)
    actions = []

    if use_velodyne_packets:
        try:
            velodyne_share = get_package_share_directory('velodyne_pointcloud')
        except PackageNotFoundError as exc:
            raise RuntimeError(
                'config_file requests /velodyne_packets, but package '
                '`velodyne_pointcloud` is not installed or not sourced. '
                'Install the ROS Velodyne stack, then source that environment '
                'before running this launch.'
            ) from exc

        calibration = velodyne_calibration or _find_velodyne_calibration_file(
            velodyne_share, scan_line
        )
        if not calibration:
            raise RuntimeError(
                'Unable to infer a Velodyne calibration file. '
                'Pass velodyne_calibration:=/abs/path/to/calibration.yaml.'
            )

        actions.append(
            Node(
                package='velodyne_pointcloud',
                executable='velodyne_transform_node',
                name='velodyne_convert',
                output='screen',
                parameters=[{
                    'calibration': calibration,
                    'model': velodyne_model,
                    'organize_cloud': True,
                    'min_range': 0.9,
                    'max_range': 130.0,
                }],
                remappings=[
                    ('velodyne_packets', lidar_topic),
                    ('velodyne_points', velodyne_points_topic),
                ],
            )
        )
        fast_lio_parameters.append({'common.lid_topic': velodyne_points_topic})

    if camera_republish:
        actions.append(
            Node(
                package='image_transport',
                executable='republish',
                name='camera_image_republish',
                arguments=['compressed', 'raw'],
                remappings=[
                    ('in/compressed', camera_compressed_topic),
                    ('out', camera_raw_topic),
                ],
            )
        )

    actions.append(
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            parameters=fast_lio_parameters,
            output='screen'
        )
    )
    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': use_sim_time.lower() == 'true'}],
            condition=IfCondition(rviz_use)
        )
    )
    return actions


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    velodyne_auto_convert = LaunchConfiguration('velodyne_auto_convert')
    velodyne_model = LaunchConfiguration('velodyne_model')
    velodyne_calibration = LaunchConfiguration('velodyne_calibration')
    velodyne_points_topic = LaunchConfiguration('velodyne_points_topic')
    camera_republish = LaunchConfiguration('camera_republish')
    camera_compressed_topic = LaunchConfiguration('camera_compressed_topic')
    camera_raw_topic = LaunchConfiguration('camera_raw_topic')
    local_ba_weight_set = LaunchConfiguration('local_ba_weight_set')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_velodyne_auto_convert_cmd = DeclareLaunchArgument(
        'velodyne_auto_convert', default_value='true',
        description='Auto-launch velodyne packet to pointcloud conversion when lid_topic is /velodyne_packets'
    )
    declare_velodyne_model_cmd = DeclareLaunchArgument(
        'velodyne_model', default_value='32C',
        description='Velodyne model used by velodyne_transform_node'
    )
    declare_velodyne_calibration_cmd = DeclareLaunchArgument(
        'velodyne_calibration', default_value='',
        description='Absolute path to Velodyne calibration yaml; auto-detected when empty'
    )
    declare_velodyne_points_topic_cmd = DeclareLaunchArgument(
        'velodyne_points_topic', default_value='/velodyne_points',
        description='PointCloud2 topic produced by the automatic Velodyne converter'
    )
    declare_camera_republish_cmd = DeclareLaunchArgument(
        'camera_republish', default_value='true',
        description='Auto-launch image_transport republish to decompress the recorded camera feed for RViz'
    )
    declare_camera_compressed_topic_cmd = DeclareLaunchArgument(
        'camera_compressed_topic', default_value='/left_camera/image/compressed',
        description='Input CompressedImage topic (base topic, without the /compressed suffix, is used as the "in" transport root)'
    )
    declare_camera_raw_topic_cmd = DeclareLaunchArgument(
        'camera_raw_topic', default_value='/camera/color/image_raw',
        description='Output raw Image topic that RViz subscribes to'
    )
    declare_local_ba_weight_set_cmd = DeclareLaunchArgument(
        'local_ba_weight_set', default_value='custom',
        description='Step-2A IMU weights: custom (YAML), A=1:1:1, B=3:1:2, or C=6:1:3. '
                    'A/B/C disable map and EKF feedback and select a matching CSV.'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_velodyne_auto_convert_cmd)
    ld.add_action(declare_velodyne_model_cmd)
    ld.add_action(declare_velodyne_calibration_cmd)
    ld.add_action(declare_velodyne_points_topic_cmd)
    ld.add_action(declare_camera_republish_cmd)
    ld.add_action(declare_camera_compressed_topic_cmd)
    ld.add_action(declare_camera_raw_topic_cmd)
    ld.add_action(declare_local_ba_weight_set_cmd)
    ld.add_action(OpaqueFunction(function=_launch_setup))

    return ld
