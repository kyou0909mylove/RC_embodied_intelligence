# -*- coding: utf-8 -*-
"""新增地面和容器的只读参数检查；拒绝把源码里的零模板作为真实开门路径。"""
import math


def finite(value):
    return type(value) in (int,float) and math.isfinite(value)


def nonzero_pose(pose):
    return isinstance(pose,list) and len(pose)==6 and all(finite(v) for v in pose) and any(pose)


def vector(value,size):
    return isinstance(value,list) and len(value)==size and all(finite(v) for v in value)


def fingers(value,closed=False):
    return vector(value,3) and all(0<=v<=100 for v in value) and (not closed or any(value))


def validate_storage(point,errors):
    storage = point.get('storage')
    name = point['id']+'.storage'
    if not isinstance(storage,dict) or storage.get('kind') not in ('cabinet','drawer'):
        errors.append(name+'.kind 须为 cabinet 或 drawer')
        return
    expected = 'shelf' if storage['kind']=='cabinet' else 'ground'
    if point['grasp_profile']!=expected:
        errors.append(name+' 柜内复用shelf，抽屉复用ground')
    for key in ('handle_pose','pregrasp_pose','clear_pose'):
        if not nonzero_pose(storage.get(key)):
            errors.append(name+'.'+key+' 须填实测非零米/度姿态；catch_3中的全零值尚未标定')
    path = storage.get('open_path')
    minimum = 4 if storage['kind']=='cabinet' else 3
    if not isinstance(path,list) or len(path)<minimum or any(not nonzero_pose(p) for p in path):
        errors.append(name+'.open_path 须填至少'+str(minimum)+'个实测非零姿态，柜门圆弧/抽屉滑轨')
    for key in ('finger_open','finger_grasp'):
        value = storage.get(key)
        if not fingers(value,closed=key=='finger_grasp'):
            errors.append(name+'.'+key+' 须为三个0–100夹爪百分比，抓把手不能全零')
        elif key=='finger_open' and max(value)>10:
            errors.append(name+'.finger_open 须在0–10百分比内以确认松开把手')
    if not finite(storage.get('timeout_s')) or storage['timeout_s']<=0:
        errors.append(name+'.timeout_s 须为有限正秒数')
    if storage['kind']=='drawer':
        geometry = point.get('ground',{})
        if (not isinstance(geometry,dict) or not vector(geometry.get('anchor_m'),3)
                or not finite(geometry.get('grasp_z_m'))):
            errors.append(point['id']+'.ground 须单独填写抽屉 anchor_m 和 grasp_z_m；禁止沿用地面高度')
        if not nonzero_pose(point.get('detect_pose')):
            errors.append(point['id']+'.detect_pose 须单独填写已打开抽屉的观察姿态')


def validate_ground_point(point,profile,errors):
    name = point['id']
    if not nonzero_pose(point.get('detect_pose')):
        errors.append(name+' 地面/抽屉必须配置有效观察姿态')
    override = point.get('ground',{})
    if not isinstance(override,dict):
        errors.append(name+'.ground 须为对象')
        return
    allowed = {'anchor_m','grasp_z_m'}
    if set(override)-allowed:
        errors.append(name+'.ground 仅接受 anchor_m/grasp_z_m，其他几何参数在profiles.ground中填')
    if 'anchor_m' in override and not vector(override['anchor_m'],3):
        errors.append(name+'.ground.anchor_m 须为三个米制数值')
    if 'grasp_z_m' in override and not finite(override['grasp_z_m']):
        errors.append(name+'.ground.grasp_z_m 须为机械臂基座坐标的有限米制Z')
    for pose in [point.get('detect_pose')]+point.get('alternate_detect_poses',[]):
        reference = profile.get('reference_detect_pose')
        if nonzero_pose(pose) and nonzero_pose(reference) and any(abs(a-b)>.01 for a,b in zip(pose[3:],reference[3:])):
            errors.append(name+' 地面观察朝向须沿用ground.reference_detect_pose；任意朝向需适配坐标公式')


def validate_ground_profile(profile,errors):
    if not isinstance(profile,dict):
        errors.append('缺少profiles.ground')
        return
    for key,size in [('anchor_m',3),('grasp_euler_xy_deg',2)]:
        if not vector(profile.get(key),size):
            errors.append('ground.'+key+' 数值/长度不符')
    if not nonzero_pose(profile.get('reference_detect_pose')):
        errors.append('ground.reference_detect_pose 须为源观察姿态')
    for key in ('grasp_z_m','yaw_offset_deg'):
        if not finite(profile.get(key)):
            errors.append('ground.'+key+' 须为有限数')
    for key in ('side_clearance_m','forward_offset_m'):
        if not finite(profile.get(key)) or profile[key]<=0:
            errors.append('ground.'+key+' 须为有限正米数')
    if not finite(profile.get('observe_settle_s', 3.0)) or profile.get('observe_settle_s', 3.0) < 0:
        errors.append('ground.observe_settle_s 须为有限非负秒数')
    for key in ('finger_open','finger_close','finger_tight'):
        if not fingers(profile.get(key),closed=key!='finger_open'):
            errors.append('ground.'+key+' 须为三个0–100百分比')
