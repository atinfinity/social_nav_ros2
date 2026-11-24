"""bringup のファイル検査(ROS 不要): BT XML に数値の判断条件が無いこと、params と launch の整合。"""
import os
import re
import xml.etree.ElementTree as ET

import yaml

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


def _params():
    with open(os.path.join(ROOT, 'params', 'nav2_social_params.yaml')) as f:
        return yaml.safe_load(f)


def test_bt_custom_nodes_only_use_wiring_attributes():
    tree = ET.parse(os.path.join(ROOT, 'behavior_trees', 'navigate_social.xml'))
    allowed = {'SelectSocialBehavior': {'path', 'behavior', 'people_topic'},
               'HoldPosition': {'behavior'}}
    found = set()
    for el in tree.iter():
        if el.tag in allowed:
            found.add(el.tag)
            assert set(el.attrib) <= allowed[el.tag], el.attrib
            for v in el.attrib.values():
                assert not re.fullmatch(r'-?[0-9.]+', v), 'numeric condition in BT XML: ' + v
    assert found == set(allowed)


def test_bt_plugin_libs_only_social_nodes():
    libs = _params()['bt_navigator']['ros__parameters']['plugin_lib_names']
    assert libs == ['social_nav_bt_nodes']


def test_plugins_declared():
    doc = _params()
    ctrl = doc['controller_server']['ros__parameters']
    assert 'SocialCritic' in ctrl['FollowPath']['critics']
    assert ctrl['SocialDWA']['plugin'] == 'social_nav_controller::SocialController'
    g = doc['global_costmap']['global_costmap']['ros__parameters']
    assert g['plugins'] == ['static_layer', 'voxel_layer', 'inflation_layer', 'social_layer']
    assert g['social_layer']['enabled'] is True
    loc = doc['local_costmap']['local_costmap']['ros__parameters']
    assert loc['social_layer']['enabled'] is False


def test_default_yaml_keeps_accel_rollout_off():
    ctrl = _params()['controller_server']['ros__parameters']['SocialDWA']
    assert not ctrl.get('accel_rollout', False)
    assert ctrl['accel_clear_time'] == 0.0


def _launch_module():
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'social_nav_launch', os.path.join(ROOT, 'launch', 'social_nav.launch.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_launch_rejects_unimplemented_combinations():
    m = _launch_module()
    m.validate('diff_drive', 'circle', 0.3, '[]')
    for args in (('omni', 'circle', 0.3, '[]'), ('ackermann', 'circle', 0.3, '[]'),
                 ('diff_drive', 'polygon', 0.3, '[[0,0]]')):
        try:
            m.validate(*args)
        except RuntimeError:
            continue
        raise AssertionError('accepted ' + str(args))


def test_launch_distributes_robot_model():
    m = _launch_module()
    doc = _params()
    m.apply_robot_model(doc, 'diff_drive', 'circle', 0.42, '[]')
    ctrl = doc['controller_server']['ros__parameters']
    assert ctrl['FollowPath']['SocialCritic']['robot_radius'] == 0.42
    assert ctrl['SocialDWA']['robot_radius'] == 0.42
    for c in ('local_costmap', 'global_costmap'):
        assert doc[c][c]['ros__parameters']['robot_radius'] == 0.42


def test_launch_collision_monitor_follows_robot_radius():
    m = _launch_module()
    doc = _params()
    m.apply_robot_model(doc, 'diff_drive', 'circle', 0.5, '[]')
    mon = doc['collision_monitor']['ros__parameters']
    assert abs(mon['PolygonStop']['radius'] - 0.65) < 1e-9
    assert abs(mon['PolygonSlow']['radius'] - 1.0) < 1e-9
    doc = _params()
    m.apply_robot_model(doc, 'diff_drive', 'circle', 0.3, '[]')
    mon = doc['collision_monitor']['ros__parameters']
    assert abs(mon['PolygonStop']['radius'] - 0.45) < 1e-9
    assert abs(mon['PolygonSlow']['radius'] - 0.8) < 1e-9


def test_collision_monitor_matches_sim_constants():
    """sim の --safety-mode cm / cm_stop の定数(social_nav_core/sim/safety.hpp)が yaml と launch の値と一致する。"""
    path = os.path.join(ROOT, '..', '..', 'social_nav_core', 'sim', 'safety.hpp')
    assert os.path.exists(path), f'social_nav_core/sim/safety.hpp not found: {path}'
    with open(path) as f:
        src = f.read()

    def const(name):
        mt = re.search(r'constexpr double ' + name + r'\s*=\s*([0-9.]+);', src)
        assert mt, f'{name} not found in safety.hpp'
        return float(mt.group(1))

    m = _launch_module()
    doc = _params()
    r = 0.3
    m.apply_robot_model(doc, 'diff_drive', 'circle', r, '[]')
    mon = doc['collision_monitor']['ros__parameters']
    assert abs(mon['PolygonStop']['radius'] - r - const('kCmStopClearance')) < 1e-9
    assert abs(mon['PolygonSlow']['radius'] - r - const('kCmSlowClearance')) < 1e-9
    assert abs(mon['PolygonSlow']['slowdown_ratio'] - const('kCmSlowRatio')) < 1e-9
    assert mon['PolygonStop']['action_type'] == 'stop'
    assert mon['PolygonSlow']['action_type'] == 'slowdown'
