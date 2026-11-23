"""social_nav の Nav2 構成を起動する(localization なし: map->odom の TF は外から供給する)。

ロボット固有の値は launch 引数で受け、全ノードの該当パラメータへ配る(設計書 10.4)。
MVP で実装済みの組み合わせは drive_type=diff_drive / footprint_type=circle だけ。
それ以外は launch 時にエラーにする。
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SUPPORTED_DRIVE = ('diff_drive',)
SUPPORTED_FOOTPRINT = ('circle',)
# Nav2 のライフサイクル管理対象(コストマップは controller/planner が内部で管理する)
LIFECYCLE_NODES = [
    'map_server', 'controller_server', 'planner_server', 'behavior_server',
    'bt_navigator', 'velocity_smoother', 'collision_monitor']


def _ros_params(doc, node):
    return doc[node]['ros__parameters']


def apply_robot_model(doc, drive_type, footprint_type, robot_radius, footprint):
    """全ノードのロボットモデル関連パラメータを 1 か所の引数から書き換える。"""
    for costmap in ('local_costmap', 'global_costmap'):
        p = _ros_params(doc[costmap], costmap)
        if footprint_type == 'circle':
            p['robot_radius'] = robot_radius
            p.pop('footprint', None)
        else:
            p['footprint'] = footprint
    controller = _ros_params(doc, 'controller_server')
    for scope in (controller['FollowPath']['SocialCritic'], controller['SocialDWA']):
        scope['footprint_type'] = footprint_type
        scope['robot_radius'] = robot_radius
        scope['footprint'] = footprint
    controller['SocialDWA']['motion_model'] = drive_type
    # collision_monitor の円は robot_radius に連動(既定 0.3 で 0.45 / 0.8)
    monitor = _ros_params(doc, 'collision_monitor')
    monitor['PolygonStop']['radius'] = robot_radius + 0.15
    monitor['PolygonSlow']['radius'] = robot_radius + 0.5


def validate(drive_type, footprint_type, robot_radius, footprint):
    if drive_type not in SUPPORTED_DRIVE:
        raise RuntimeError(
            "drive_type '%s' is not implemented (supported: %s)" % (drive_type, ', '.join(SUPPORTED_DRIVE)))
    if footprint_type not in SUPPORTED_FOOTPRINT:
        raise RuntimeError(
            "footprint_type '%s' is not implemented (supported: %s)"
            % (footprint_type, ', '.join(SUPPORTED_FOOTPRINT)))
    if robot_radius <= 0.0:
        raise RuntimeError('robot_radius must be positive')


def _setup(context):
    cfg = {k: LaunchConfiguration(k).perform(context) for k in (
        'drive_type', 'footprint_type', 'robot_radius', 'footprint', 'params_file', 'map',
        'bt_xml', 'controller', 'use_sim_time', 'autostart', 'people_tracker')}
    radius = float(cfg['robot_radius'])
    validate(cfg['drive_type'], cfg['footprint_type'], radius, cfg['footprint'])
    if cfg['controller'] not in ('mppi', 'social'):
        raise RuntimeError("controller must be 'mppi' or 'social'")

    with open(cfg['params_file']) as f:
        doc = yaml.safe_load(f)
    apply_robot_model(doc, cfg['drive_type'], cfg['footprint_type'], radius, cfg['footprint'])
    _ros_params(doc, 'map_server')['yaml_filename'] = cfg['map']

    bt_xml = cfg['bt_xml']
    if cfg['controller'] == 'social':
        # 比較用: FollowPath の controller_id だけを SocialDWA に差し替えた複製を使う
        with open(bt_xml) as f:
            text = f.read()
        if 'controller_id="FollowPath"' not in text:
            raise RuntimeError('controller_id="FollowPath" not found in ' + bt_xml)
        text = text.replace('controller_id="FollowPath"', 'controller_id="SocialDWA"')
        fd, bt_xml = tempfile.mkstemp(prefix='navigate_social_', suffix='.xml')
        with os.fdopen(fd, 'w') as f:
            f.write(text)
    _ros_params(doc, 'bt_navigator')['default_nav_to_pose_bt_xml'] = bt_xml

    fd, params_path = tempfile.mkstemp(prefix='nav2_social_params_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(doc, f)

    sim = {'use_sim_time': cfg['use_sim_time'] == 'true'}
    params = [params_path, sim]
    lifecycle_cmd = [('cmd_vel', 'cmd_vel_nav')]
    nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=params),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=params, remappings=lifecycle_cmd),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=params),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=params),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=params),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=params,
             remappings=lifecycle_cmd),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', parameters=params),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[params_path, sim, {'autostart': cfg['autostart'] == 'true',
                                            'node_names': LIFECYCLE_NODES}]),
    ]
    if cfg['people_tracker'] == 'true':
        nodes.append(Node(package='social_nav_perception', executable='people_tracker_node',
                          name='people_tracker_node', output='screen', parameters=params))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('social_nav_bringup')
    args = [
        ('drive_type', 'diff_drive', '駆動方式 (diff_drive のみ実装)'),
        ('footprint_type', 'circle', 'フットプリント (circle のみ実装)'),
        ('robot_radius', '0.3', 'circle の半径 [m]'),
        ('footprint', '[]', '多角形フットプリント "[[x,y],...]" (polygon は未実装)'),
        ('params_file', os.path.join(share, 'params', 'nav2_social_params.yaml'), 'Nav2 パラメータ'),
        ('map', os.path.join(share, 'maps', 'empty.yaml'), '地図 yaml'),
        ('bt_xml', os.path.join(share, 'behavior_trees', 'navigate_social.xml'), 'BT XML'),
        ('controller', 'mppi', 'mppi (MPPI + SocialCritic) | social (SocialController、比較用)'),
        ('use_sim_time', 'false', 'シミュレーション時刻'),
        ('autostart', 'true', 'lifecycle を自動で active にする'),
        ('people_tracker', 'true', 'people_tracker_node を起動する'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, desc in args]
        + [OpaqueFunction(function=_setup)])
