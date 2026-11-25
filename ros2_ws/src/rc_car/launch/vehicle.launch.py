import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rc_car.core import load_config, odom_configured, require


def launch_nodes(context):
    path = LaunchConfiguration('config').perform(context)
    cfg = load_config(path)
    actions = []
    wheel = cfg['geometry']['odom_source']=='wheel'
    executables = ['command_manager', 'vehicle_controller', 'web_bridge']
    if wheel:
        executables.append('odometry')
    for executable in executables:
        node = Node(package='rc_car', executable=executable, output='screen', parameters=[{'config': path, 'use_sim_time': False}])
        actions.append(node)
        actions.append(RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[EmitEvent(event=Shutdown(reason=f'{executable} exited'))])))
    lidar = cfg['lidar']
    if lidar['mount_verified']:
        args = []
        for name, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), lidar['xyz_rpy']):
            args += ['--'+name, str(value)]
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            arguments=args + ['--frame-id', 'base_link', '--child-frame-id', lidar['frame']], output='screen'))
    if lidar['enabled']:
        launch = os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_c1_launch.py')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(launch), launch_arguments={
            'serial_port': lidar['serial_port'], 'serial_baudrate': str(lidar['serial_baudrate']),
            'frame_id': lidar['frame'], 'inverted': 'false', 'angle_compensate': 'true'}.items()))
    if not wheel and odom_configured(cfg):
        # RTAB-Map core parameters are strings; native ROS parameters are typed.
        icp_params = {key: ParameterValue(value, value_type=str) for key, value in {
            'Reg/Force3DoF': 'true', 'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneLowComplexityStrategy': '0', 'Odom/ResetCountdown': '0',
        }.items()}
        icp = Node(package='rtabmap_odom', executable='icp_odometry', output='screen',
                   parameters=[dict(icp_params, frame_id='base_link', odom_frame_id='odom',
                                    publish_tf=True, publish_null_when_lost=False, qos=2,
                                    wait_imu_to_init=False, use_sim_time=False)])
        actions.append(icp)
        actions.append(RegisterEventHandler(OnProcessExit(target_action=icp, on_exit=[
            EmitEvent(event=Shutdown(reason='lidar odometry exited'))])))
    if cfg['slam']['enabled']:
        require(odom_configured(cfg) and lidar['enabled'] and lidar['mount_verified'], 'SLAM requires configured odometry source and measured lidar TF')
        with open(cfg['slam']['params_file'], encoding='utf-8') as stream:
            parameters = yaml.safe_load(stream)
        settings = parameters['slam_toolbox']['ros__parameters']
        settings.update(use_sim_time=False, odom_frame='odom', map_frame='map', base_frame='base_link', scan_topic='/scan', use_map_saver=True)
        require(settings.get('transform_publish_period', 0.02) > 0, 'existing slam.yaml disables map->odom TF')
        localization = cfg['slam']['localization']
        settings['mode'] = 'localization' if localization else 'mapping'
        if localization:
            graph = cfg['slam']['posegraph']
            require(graph and os.path.isfile(graph + '.posegraph') and os.path.isfile(graph + '.data'), 'serialized pose graph required')
            settings['map_file_name'] = graph
            settings['map_start_at_dock'] = True
        else:
            # Do not accidentally load an old graph referenced by the existing YAML.
            settings.pop('map_file_name', None)
            settings.pop('map_start_pose', None)
            settings.pop('map_start_at_dock', None)
        fd, generated = tempfile.mkstemp(prefix='rc_slam_', suffix='.yaml')
        with os.fdopen(fd, 'w') as stream:
            yaml.safe_dump(parameters, stream)
        launch = os.path.join(get_package_share_directory('slam_toolbox'), 'launch',
                              'localization_launch.py' if localization else 'online_async_launch.py')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(launch), launch_arguments={
            'slam_params_file': generated, 'use_sim_time': 'false', 'autostart': 'true'}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config', default_value='/project/config/vehicle.yaml'),
                              OpaqueFunction(function=launch_nodes)])
