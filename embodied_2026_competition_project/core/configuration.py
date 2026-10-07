# -*- coding: utf-8 -*-
"""主程序配置校验与旧配置迁移；只用标准库，不连接设备。

现场参数统一写入 config/navigation.local.json。缺省值来自新模板及 arm/defaults.py。
地图点位保持空值，禁止用往年地图坐标代替现场点位。
"""
from .timeouts import apply_defaults, validate as validate_timeouts
from dataclasses import dataclass
import copy
import hashlib
import json
import math
from pathlib import Path
import re
from arm.defaults import HOME_POSE, PLACE_POSE, SURFACE_POSE, SHELF_LAYERS
from .storage_configuration import validate_storage, validate_ground_point, validate_ground_profile


class ConfigurationRejected(ValueError):
    pass


@dataclass
class Configuration:
    root: Path
    path: Path
    values: dict
    model_path: Path
    evidence_path: Path
    labels: list


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _valid_arm_pose(pose):
    return isinstance(pose, list) and len(pose) == 6 and all(_number(v) for v in pose)


def _pose_error(pose):
    if (not isinstance(pose, list) or len(pose) != 2 or
            not all(isinstance(p, list) for p in pose) or
            len(pose[0]) != 3 or len(pose[1]) != 4 or
            not all(_number(x) for row in pose for x in row)):
        return '须为 [[x,y,0],[0,0,qz,qw]]，位置单位米'
    if abs(sum(x*x for x in pose[1]) - 1) > 0.02:
        return '四元数未归一化'
    if abs(pose[0][2]) > .01 or abs(pose[1][0]) > .01 or abs(pose[1][1]) > .01:
        return '平面地图导航使用 z=0、qx=qy=0'
    return None


def _merge(default, supplied):
    merged = copy.deepcopy(default)
    for key, value in supplied.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _resolved(value, parent):
    candidate = Path(value).expanduser()
    return (candidate if candidate.is_absolute() else parent/candidate).resolve()


def _validate_model(model, name, parent, root, errors):
    if not isinstance(model, dict):
        errors.append(name + ' 须为对象')
        return None
    labels = model.get('labels')
    if (not isinstance(labels, list) or not labels or
            any(not isinstance(s, str) or not s for s in labels) or len(set(labels)) != len(labels)):
        errors.append(name + '.labels 须为精确、唯一的模型类别列表')
    weight = model.get('weights')
    if not isinstance(weight, str) or not weight:
        errors.append(name + '.weights 未填写')
        return None
    file = _resolved(weight, parent)
    model['weights'] = str(file)
    if not file.is_file() or hashlib.sha256(file.read_bytes()).hexdigest() != model.get('sha256'):
        errors.append(name + ' 权重不存在或 SHA256 不符')
    try:
        info_name = model.get('metadata_file', str(root/'models/TABLE_MODEL_INFO.json'))
        info = json.loads(_resolved(info_name, parent).read_text(encoding='utf-8'))
        names = [v for _, v in sorted(info['names'].items(), key=lambda pair: int(pair[0]))]
        if labels != names or info['sha256'] != model.get('sha256'):
            errors.append(name + ' 标签/哈希与模型元数据不符')
        model['metadata_file'] = str(_resolved(info_name, parent))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        errors.append(name + ' 元数据读取失败：' + str(exc))
    if not _number(model.get('confidence')) or not 0 < model['confidence'] <= 1:
        errors.append(name + '.confidence 须在 (0,1] 内')
    return file


def _flow_defaults(cfg):
    """新流程参数集中补默认值；保留现场已填写的坐标、标定和机械臂姿态。"""
    for group, defaults in {
        'localization': {'navigate_to_initial_pose': True},
        'strategy': {'empty_confirmations': 2, 'round_wait_s': 3,
                     'max_visits_per_point': 0},
        'recovery': {'point_wait_timeout_s': 120, 'reset_timeout_s': 30,
                     'drop_retry_wait_s': 5, 'drop_retry_timeout_s': 120,
                     'costmap_timeout_s': 3, 'costmap_settle_s': 1},
    }.items():
        if not isinstance(cfg.get(group), dict):
            continue  # 格式错误留给配置校验生成具体提示。
        for key, value in defaults.items():
            cfg[group].setdefault(key, value)
    actions = cfg.get('actions')
    profiles = actions.get('profiles') if isinstance(actions, dict) else None
    ground = profiles.get('ground') if isinstance(profiles, dict) else None
    if isinstance(ground, dict):
        ground.setdefault('angle_fallback_deg', 0.0)
    # 旧冷却/次数/预算字段不再参与顺序逐轮调度，删除避免现场误调。
    for key in ('empty_rechecks_per_point','empty_recheck_delay_s','blocked_cooldown_s','level_timeout_s'):
        if isinstance(cfg.get('strategy'),dict):
            cfg['strategy'].pop(key,None)
    for key in ('nav_estimate_s','scan_estimate_s','grasp_estimate_s','place_estimate_s'):
        if isinstance(cfg.get('timing'),dict):
            cfg['timing'].pop(key,None)
    if isinstance(cfg.get('localization'),dict):
        cfg['localization'].pop('confirmation_timeout_s',None)
    return apply_defaults(cfg)


def load_runtime_configuration(path, root):
    """普通启动只读取参数、补默认值和解析路径；完整预检由 --check-config 调用。

    不计算权重哈希、不读取模型元数据、不逐项检查所有家具和数值范围。
    模型类别使用实际加载的权重，点位和动作参数在真正使用时读取。
    """
    path, root = Path(path).expanduser().resolve(), Path(root).resolve()
    try:
        supplied = json.loads(path.read_text(encoding='utf-8'))
        template = json.loads((root/'config/navigation.example.json').read_text(encoding='utf-8'))
        cfg = _flow_defaults(_merge(template, supplied))
    except (OSError, ValueError, TypeError, AttributeError) as exc:
        raise ConfigurationRejected('配置读取失败：' + str(exc)) from exc
    old_version = supplied.get('schema_version', 7)
    cfg['schema_version'] = 7
    cfg.setdefault('profile', 'competition')
    cfg.pop('start', None)
    cfg.pop('interference', None)
    for key in ('workspace_xyz_m', 'feedback', 'put_waypoints',
                'calibration_verified', 'initially_empty_confirmed'):
        cfg['actions'].pop(key, None)
    for key in ('enabled', 'max_attempts_per_point', 'successes_before_switch', 'level_timeout_s'):
        cfg['strategy'].pop(key, None)
    for key in ('empty_detection', 'exit_when_done'):
        cfg['flow'].pop(key, None)
    for key, default in [('home_pose', HOME_POSE), ('place_pose', PLACE_POSE)]:
        if cfg['actions'].get(key) is None:
            cfg['actions'][key] = copy.deepcopy(default)
    for index, point in enumerate(cfg['detection_points'], 1):
        point.setdefault('id', 'detect_{:02d}'.format(index))
        point.setdefault('name', point['id'])
        point.setdefault('region', 'room')
        point.setdefault('furniture_id', point['id'])
        point.setdefault('difficulty', 1)
        point.setdefault('enabled', True)
        level = point['difficulty']
        if old_version < 7 and level == 3:
            point['enabled'], point['grasp_profile'] = False, 'shelf'
        point.setdefault('grasp_profile', 'surface' if level == 1 else 'shelf')
        if old_version < 6:
            point['target_labels'] = []
        point.setdefault('target_labels', [])
        point.setdefault('alternate_detect_poses', [])
        point.setdefault('reposition_points', [])
        point.setdefault('lift_dz_m', .03)
        if not point['enabled']:
            continue
        if point['grasp_profile'] == 'surface' and point.get('detect_pose') is None:
            point['detect_pose'] = copy.deepcopy(SURFACE_POSE)
        elif point['grasp_profile'] == 'shelf':
            if not point.get('layers'):
                point['layers'] = copy.deepcopy(SHELF_LAYERS)
            if point.get('layer_hint') is None:
                point['layer_hint'] = point['layers'][1]['id']
            for layer in point['layers']:
                layer.setdefault('alternate_detect_poses', [])
        elif (point['grasp_profile'] == 'ground' and level == 2
              and point.get('detect_pose') is None):
            point['detect_pose'] = copy.deepcopy(
                cfg['actions']['profiles']['ground']['reference_detect_pose'])
    for model in [cfg['model']] + [p['model'] for p in cfg['actions']['profiles'].values()]:
        model['weights'] = str(_resolved(model['weights'], path.parent))
    model_path = Path(cfg['model']['weights'])
    evidence_path = _resolved(cfg['evidence_dir'], path.parent)
    return Configuration(root, path, cfg, model_path, evidence_path, cfg['model']['labels'])


def load_configuration(path, root, require_poses=True):
    path, root = Path(path).expanduser().resolve(), Path(root).resolve()
    try:
        supplied = json.loads(path.read_text(encoding='utf-8'))
        template = json.loads((root/'config/navigation.example.json').read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise ConfigurationRejected('配置读取失败：' + str(exc)) from exc
    if not isinstance(supplied, dict) or supplied.get('schema_version') not in (4, 5, 6, 7):
        raise ConfigurationRejected('支持 schema_version=4/5/6/7；新模板使用7')
    cfg = _flow_defaults(_merge(template, supplied))
    old_version = supplied['schema_version']
    cfg['schema_version'], cfg['profile'] = 7, 'competition'
    errors = []
    for key in ('actions', 'model', 'camera', 'timing', 'flow', 'strategy', 'speech', 'localization', 'perception', 'recording', 'recovery'):
        if not isinstance(cfg.get(key), dict):
            errors.append(key + ' 须为对象')
    if errors:
        raise ConfigurationRejected('\n'.join(errors))
    for key in ('wrist','gripper_feedback','profiles'):
        if not isinstance(cfg['actions'].get(key),dict):
            errors.append('actions.'+key+' 须为对象')
    if errors:
        raise ConfigurationRejected('\n'.join(errors))
    for kind in ('surface','shelf','ground'):
        if not isinstance(cfg['actions']['profiles'].get(kind),dict):
            errors.append('actions.profiles.'+kind+' 须为对象')
    if errors:
        raise ConfigurationRejected('\n'.join(errors))
    # 已取消开始信号、人员监测、ROI、末端范围和中间释放姿态，旧字段不再生效。
    cfg.pop('start', None)
    cfg.pop('interference', None)
    cfg['flow'].pop('empty_detection', None)
    cfg['flow'].pop('exit_when_done', None)
    for key in ('workspace_xyz_m', 'feedback', 'put_waypoints', 'calibration_verified', 'initially_empty_confirmed'):
        cfg['actions'].pop(key, None)
    cfg['strategy'].pop('enabled', None)
    cfg['strategy'].pop('max_attempts_per_point', None)
    cfg['strategy'].pop('successes_before_switch', None)
    cfg['strategy'].pop('level_timeout_s', None)  # 顺序路线不再使用难度时间窗。
    locations = cfg.get('locations')
    if not isinstance(locations, dict):
        raise ConfigurationRejected('locations 须为对象')
    map_poses = [('locations.'+key, locations.get(key)) for key in ('entry', 'drop', 'exit')]
    if cfg.get('initial_pose') is not None:
        map_poses.append(('initial_pose', cfg['initial_pose']))
    points = cfg.get('detection_points')
    if not isinstance(points, list) or not points or any(not isinstance(p, dict) for p in points):
        raise ConfigurationRejected('detection_points 须为非空对象列表')
    ids = [p.get('id') for p in points]
    if any(not isinstance(s, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', s) for s in ids) or len(set(ids)) != len(ids):
        raise ConfigurationRejected('检测点 id 无效或重复')
    labels = cfg['model'].get('labels', [])
    if not isinstance(labels, list) or any(not isinstance(label, str) for label in labels):
        raise ConfigurationRejected('model.labels 须为字符串列表')
    for point in points:
        pid = point['id']
        for key in ('roi', 'wrist_roi', 'surface_verified'):
            point.pop(key, None)
        point.setdefault('name', pid)
        point.setdefault('region', 'room')
        point.setdefault('furniture_id', pid)
        point.setdefault('difficulty', 1)
        point.setdefault('enabled',True)
        point.setdefault('reposition_points',[])
        if type(point['enabled']) is not bool:
            errors.append(pid+'.enabled 须为布尔值')
        level = point['difficulty']
        if type(level) is not int or level not in (1, 2, 3):
            errors.append(pid + '.difficulty 使用1/2/3')
            continue
        if old_version<7 and level==3:
            point['enabled'] = False  # 旧三级仅接口，不能迁移成自动开门。
            point['grasp_profile'] = 'shelf'
        point.setdefault('grasp_profile', {1: 'surface', 2: 'shelf', 3: 'shelf'}[level])
        allowed = {1:('surface',),2:('shelf','ground'),3:('shelf','ground')}[level]
        if point['grasp_profile'] not in allowed:
            errors.append(pid + ' 抓取方式与难度不符')
        if not point['enabled']:
            continue  # 预留的未标定点不发动作，也不要求其地图位姿。
        for key in ('name', 'region', 'furniture_id'):
            if not isinstance(point[key], str) or not point[key]:
                errors.append(pid + '.' + key + ' 须为非空字符串')
        map_poses.append((pid+'.pose', point.get('pose')))
        # 原模板曾强制写单个目标；迁移时恢复随机摆放的全部类别识别。
        if old_version < 6:
            point['target_labels'] = []
        point.setdefault('target_labels', [])
        selected = point['target_labels']
        if not isinstance(selected, list) or any(s not in labels for s in selected):
            errors.append(pid + '.target_labels 含模型没有的类别')
        point.setdefault('alternate_detect_poses', [])
        point.setdefault('reposition_points', [])
        point.setdefault('lift_dz_m', .03)
        if not _number(point['lift_dz_m']) or not .005 <= point['lift_dz_m'] <= .15:
            errors.append(pid + '.lift_dz_m 须在 .005–.15 米')
        if not isinstance(point['alternate_detect_poses'],list):
            errors.append(pid+' 备用观察姿态须为列表')
            point['alternate_detect_poses'] = []
        if level==3:
            validate_storage(point,errors)
        if point['grasp_profile'] == 'surface':
            if point.get('detect_pose') is None:
                point['detect_pose'] = copy.deepcopy(SURFACE_POSE)
            poses = [point['detect_pose']] + point['alternate_detect_poses']
        elif point['grasp_profile'] == 'shelf':
            if not point.get('layers') and level==2:
                point['layers'] = copy.deepcopy(SHELF_LAYERS)
            layers = point.get('layers')
            if (not isinstance(layers, list) or not layers or len(layers)>3 or
                    (level==2 and len(layers)!=3) or any(not isinstance(d, dict) for d in layers)):
                errors.append(pid + '.layers 二级须为三层，柜内须填写1–3层，顺序低到高')
                continue
            layer_ids = [d.get('id') for d in layers]
            if any(not isinstance(s, str) or not s for s in layer_ids) or len(set(layer_ids)) != len(layer_ids):
                errors.append(pid + ' 柜层 id 无效或重复')
            if point.get('layer_hint') is None:
                point['layer_hint'] = layer_ids[len(layer_ids)//2]
            if point['layer_hint'] not in layer_ids:
                errors.append(pid + '.layer_hint 未定义')
            poses = []
            for layer in layers:
                layer.pop('wrist_roi', None)
                layer.setdefault('alternate_detect_poses', [])
                if not isinstance(layer['alternate_detect_poses'], list):
                    errors.append(pid + ' 柜层备用姿态须为列表')
                    continue
                poses += [layer.get('detect_pose')] + layer['alternate_detect_poses']
            # 保持源柜层公式对应的朝向；不能把它当作任意朝向的通用手眼变换。
            for pose in poses:
                expected_euler = cfg['actions']['profiles']['shelf'].get('detect_euler_deg', [])
                if _valid_arm_pose(pose) and any(abs(a-b)>0.01 for a,b in zip(pose[3:], expected_euler)):
                    errors.append(pid + ' 柜层观察朝向须沿用原值；任意朝向需另行适配坐标变换')
        elif point['grasp_profile']=='ground':
            profile = cfg['actions']['profiles']['ground']
            if point.get('detect_pose') is None and level==2:
                point['detect_pose'] = copy.deepcopy(profile['reference_detect_pose'])
            validate_ground_point(point,profile,errors)
            poses = [point.get('detect_pose')]+point['alternate_detect_poses']
        else:
            poses = []
        if not isinstance(point['alternate_detect_poses'], list):
            errors.append(pid + ' 备用观察姿态须为列表')
        if any(not _valid_arm_pose(pose) for pose in poses):
            errors.append(pid + ' 机械臂姿态须为 [x,y,z,tx,ty,tz]，米/度')
    if require_poses and cfg['localization'].get('navigate_to_initial_pose') and cfg.get('initial_pose') is None:
        errors.append('initial_pose 自动初始导航需要填写地图位姿')
    for name, pose in map_poses:
        if pose is None:
            if require_poses:
                errors.append('待填写地图位姿：' + name)
        elif _pose_error(pose):
            errors.append(name + '：' + _pose_error(pose))
    by_id = {p['id']: p for p in points}
    for point in points:
        if not point['enabled']:
            continue
        alternatives = point['reposition_points']
        if not isinstance(alternatives, list) or any(
                not isinstance(pid, str) or pid not in by_id or pid == point['id'] or
                by_id[pid]['furniture_id'] != point['furniture_id'] or
                not by_id[pid]['enabled'] or by_id[pid]['difficulty']!=point['difficulty'] or
                by_id[pid]['grasp_profile'] != point['grasp_profile'] for pid in alternatives):
            errors.append(point['id'] + '.reposition_points 须引用同一家具、同一抓取方式的其他点')
    model_path = _validate_model(cfg['model'], 'model', path.parent, root, errors)
    actions = cfg['actions']
    if actions.get('mode') not in ('kinova', 'print'):
        errors.append('actions.mode 使用 kinova 或 print')
    if actions.get('robot_type') != 'j2n6s300':
        errors.append('actions.robot_type 来源工程仅适配j2n6s300')
    for key, default in [('home_pose', HOME_POSE), ('place_pose', PLACE_POSE)]:
        if actions.get(key) is None:
            actions[key] = copy.deepcopy(default)
        if not _valid_arm_pose(actions[key]):
            errors.append('actions.' + key + ' 不是米/度六维姿态')
    if not isinstance(actions.get('tool_frame'), str) or not actions['tool_frame']:
        errors.append('actions.tool_frame 须为驱动输出坐标系')
    if not isinstance(actions.get('camera_serial'), str):
        errors.append('actions.camera_serial 使用字符串；空字符串为自动选择唯一 RealSense')
    for key in ('finger_open', 'finger_close', 'finger_tight'):
        v = actions.get(key)
        if not isinstance(v, list) or len(v) != 3 or any(not _number(n) or not 0<=n<=100 for n in v):
            errors.append('actions.'+key+' 须为三个0–100百分比')
    for kind in ('surface', 'shelf','ground'):
        profile = actions.get('profiles', {}).get(kind)
        if not isinstance(profile, dict) or profile.get('transform') != kind:
            errors.append('缺少 '+kind+' 标定/模型配置')
            continue
        matrix = profile.get('hand_eye_R')
        if not isinstance(matrix, list) or len(matrix) != 3 or any(
                not isinstance(row, list) or len(row)!=3 or any(not _number(v) for v in row) for row in matrix):
            errors.append('actions.profiles.'+kind+'.hand_eye_R 须为3x3矩阵')
        for key in (('tcp_grip_cam_m',) if kind=='surface' else
                    ('hand_eye_T_m','compensation_m') if kind=='shelf' else ('hand_eye_T_m',)):
            vec = profile.get(key)
            if not isinstance(vec, list) or len(vec)!=3 or any(not _number(v) for v in vec):
                errors.append('actions.profiles.'+kind+'.'+key+' 须为三个米制数值')
        _validate_model(profile.get('model'), 'actions.profiles.'+kind+'.model', path.parent, root, errors)
        mapping = profile.get('label_map')
        profile_model = profile.get('model')
        model_labels = profile_model.get('labels', []) if isinstance(profile_model, dict) else []
        if not isinstance(model_labels, list):
            model_labels = []
        if not isinstance(mapping, dict) or any(k not in labels or v not in model_labels for k,v in mapping.items()):
            errors.append(kind+' 标签映射须为头部类别到腕部精确类别')
    validate_ground_profile(actions['profiles'].get('ground'),errors)
    # 同一容器共享开门状态，因此不能给同一家具混填柜门和抽屉。
    storages = {}
    for point in points:
        if point['enabled'] and point['difficulty']==3 and isinstance(point.get('storage'),dict):
            previous = storages.setdefault(point['furniture_id'],point['storage'])
            if previous != point['storage']:
                errors.append(point['furniture_id']+' 同一容器的各底盘观察点须使用相同storage开门参数')
    # 全场数量只记录规则背景，不作为我方抓满数量或每个家具的库存。
    policy = cfg['strategy']
    levels = policy.get('enabled_levels')
    if not isinstance(levels, list) or not levels or any(type(n) is not int or n not in (1,2,3) for n in levels) or len(set(levels)) != len(levels):
        errors.append('strategy.enabled_levels 为 [1,2,3] 或其非空子集；三级还需启用已配置的点')
    elif not any(p['enabled'] and p['difficulty'] in levels for p in points):
        errors.append('没有已启用且属于enabled_levels的观察点')
    numeric_groups = [
        (cfg['timing'], ['match_s','exit_reserve_s','prepare_timeout_s','nav_timeout_s','scan_timeout_s','exit_estimate_s']),
        (actions, ['grasp_timeout_s','place_timeout_s','tool_pose_timeout_s','tool_pose_max_age_s','pose_tolerance_m','orientation_tolerance_deg']),
        (actions['wrist'], ['depth_min_m','depth_max_m','depth_tolerance_m','angle_tolerance_deg','max_age_s','scan_timeout_s']),
        (actions['gripper_feedback'], ['timeout_s','max_age_s','stable_span_s','stable_tolerance_turn','empty_closed_turn','blocked_margin_turn','open_max_turn']),
        (policy, ['round_wait_s']),
        (cfg['recovery'], ['retry_delay_s','point_wait_timeout_s','reset_timeout_s',
                         'drop_retry_wait_s','drop_retry_timeout_s','costmap_timeout_s','costmap_settle_s']),
    ]
    group_names = ['timing','actions','actions.wrist','actions.gripper_feedback','strategy','recovery']
    for (group,keys),group_name in zip(numeric_groups,group_names):
        for key in keys:
            if not _number(group.get(key)) or group[key]<=0:
                errors.append(group_name+'.'+key+' 须为有限正数')
    if cfg['timing'].get('match_s') != 600 or not _number(cfg['timing'].get('exit_reserve_s')) or not 0 < cfg['timing']['exit_reserve_s'] < 600:
        errors.append('整场固定600秒，离场预留须在0–600秒内')
    for point in points:
        if point['enabled'] and 'grasp_timeout_s' in point and (not _number(point['grasp_timeout_s']) or point['grasp_timeout_s']<=0):
            errors.append(point['id']+'.grasp_timeout_s 须为有限正秒数')
    for group, keys in [(policy,['empty_confirmations']), (actions['wrist'],['frames','min_hits','settle_frames','max_fresh_retries','width','height']), (actions['gripper_feedback'],['stable_samples','min_blocked_fingers']), (cfg['recovery'],['camera_restarts','nav_retries','speech_retries'])]:
        for key in keys:
            if type(group.get(key)) is not int or group[key]<0 or (key not in ('camera_restarts','nav_retries','speech_retries','max_fresh_retries') and group[key]<1):
                errors.append(key+' 须为范围内整数')
    if (type(actions['gripper_feedback'].get('min_blocked_fingers')) is not int or
            not 1<=actions['gripper_feedback']['min_blocked_fingers']<=3 or
            type(actions['gripper_feedback'].get('stable_samples')) is not int or actions['gripper_feedback']['stable_samples']<2):
        errors.append('夹爪需2个以上稳定样本，受阻指数量为1–3')
    for group in (cfg['model'], actions['wrist']):
        if group.get('device') not in ('auto','cpu') and not (isinstance(group.get('device'),str) and group['device'].isdigit()):
            errors.append('device 使用 auto、cpu 或 GPU编号字符串')
        if type(group.get('imgsz')) is not int or not 32<=group['imgsz']<=1920:
            errors.append('imgsz 须为32–1920整数')
    if type(cfg['model'].get('frames')) is not int or not 1<=cfg['model']['frames']<=20:
        errors.append('model.frames 须为1–20')
    if cfg['camera'].get('backend')!='azure_kinect' or cfg['camera'].get('color_mode')!='1080p':
        errors.append('当前头部相机实现固定为 Azure Kinect 1080p；换相机须先修改 camera/vision.py')
    for group,keys in [(cfg['perception'],['depth_tolerance_m']),
                       (cfg['speech'],['pause_s','timeout_s']),
                       (cfg['localization'],['initial_pose_wait_s'])]:
        for key in keys:
            if not _number(group.get(key)) or group[key]<=0:
                errors.append(key+' 须为有限正数')
    if type(policy.get('max_visits_per_point')) is not int or policy['max_visits_per_point'] < 0:
        errors.append('max_visits_per_point 须为非负整数；0表示不按次数结束')
    fallback = actions['profiles']['ground'].get('angle_fallback_deg')
    if fallback is not None and (not _number(fallback) or not -90 <= fallback <= 90):
        errors.append('angle_fallback_deg 须为[-90,90]度或null')
    for point in points:
        if 'wait_timeout_s' in point and (not _number(point['wait_timeout_s']) or point['wait_timeout_s'] <= 0):
            errors.append(point['id']+'.wait_timeout_s 须为有限正秒数')
    for group,keys in [(cfg['localization'],['navigate_to_initial_pose']),
                       (cfg['speech'],['announce_stages','wait_done']),
                       (cfg['flow'],['skip_unreachable_detection']),
                       (cfg['recording'],['save_images'])]:
        for key in keys:
            if type(group.get(key)) is not bool:
                errors.append(key+' 须为布尔值')
    names = cfg['speech'].get('label_names')
    if not isinstance(names,dict) or any(not isinstance(v,str) or not v for v in names.values()):
        errors.append('speech.label_names 须为类别到非空播报名称的对象')
    shelf = actions['profiles'].get('shelf',{})
    if not isinstance(shelf,dict):
        shelf = {}
    if type(shelf.get('detect_tries')) is not int or not 1<=shelf['detect_tries']<=10:
        errors.append('shelf.detect_tries 须为1–10整数')
    if not _number(shelf.get('align_y_factor')) or not 0<shelf['align_y_factor']<=1:
        errors.append('shelf.align_y_factor 须在(0,1]内')
    if (not isinstance(shelf.get('detect_euler_deg'),list) or len(shelf['detect_euler_deg'])!=3
            or not all(_number(v) for v in shelf['detect_euler_deg'])):
        errors.append('shelf.detect_euler_deg 须为三个有限角度值')
    if shelf.get('execution_mode')!='full':
        errors.append('actions.profiles.shelf.execution_mode 比赛流程须为full；align_only会停在调试位')
    feedback = actions['gripper_feedback']
    thresholds = [feedback.get(k) for k in ('open_max_turn','empty_closed_turn','blocked_margin_turn')]
    if all(_number(v) for v in thresholds):
        if not 0<thresholds[0]<thresholds[1]-thresholds[2]<thresholds[1]<=6800:
            errors.append('夹爪阈值须满足 0<open_max<empty_closed-blocked_margin<empty_closed<=6800')
    if (_number(cfg['timing'].get('exit_reserve_s')) and _number(cfg['timing'].get('exit_estimate_s'))
            and cfg['timing']['exit_reserve_s']<cfg['timing']['exit_estimate_s']):
        errors.append('离场预留不能小于离场预计用时')
    for policy2, frames in [(cfg['perception'],cfg['model']['frames']), (actions['wrist'],actions['wrist']['frames'])]:
        if type(policy2.get('min_hits')) is not int or type(frames) is not int or not 1<=policy2['min_hits']<=frames:
            errors.append('min_hits 不能超过 frames')
        if not _number(policy2.get('iou_threshold')) or not 0<policy2['iou_threshold']<1:
            errors.append('多帧 iou_threshold 须在0–1内')
    if (_number(actions['wrist'].get('depth_min_m')) and _number(actions['wrist'].get('depth_max_m'))
            and actions['wrist']['depth_min_m']>=actions['wrist']['depth_max_m']):
        errors.append('腕部深度上下限顺序错误')
    if not isinstance(cfg.get('excluded_labels'),list) or any(s not in labels for s in cfg['excluded_labels']):
        errors.append('excluded_labels 须为模型类别列表')
    if not isinstance(cfg.get('stop_topic'),str) or not cfg['stop_topic'].startswith('/'):
        errors.append('stop_topic 须为ROS绝对Bool话题')
    if not isinstance(cfg.get('evidence_dir'),str) or not cfg['evidence_dir'].strip():
        errors.append('evidence_dir 须为非空路径')
    validate_timeouts(cfg,errors)
    if errors:
        raise ConfigurationRejected('配置未完成，未连接机器人：\n- ' + '\n- '.join(errors))
    evidence_path = _resolved(cfg['evidence_dir'], path.parent)
    return Configuration(root, path, cfg, model_path, evidence_path, labels)
