# -*- coding: utf-8 -*-
"""同场恢复：保存已用时间与任务状态，重启 Python 不重新获得 600 秒。"""
import hashlib
import json
import math
import os
from pathlib import Path


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def config_digest(values):
    raw = json.dumps(values, sort_keys=True, ensure_ascii=False).encode('utf-8')
    return hashlib.sha256(raw).hexdigest()


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def read_checkpoint(path, values, strict=True):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data,dict):
        raise ValueError(str(path)+'：恢复文件根节点须为对象')
    if data.get('schema_version') != 1:
        raise ValueError('恢复文件版本不符')
    if strict and data.get('boot_id') != boot_id():
        raise ValueError('恢复文件版本不符或电脑已重启；不能可靠沿用本场单调时钟')
    if strict and data.get('config_digest') != config_digest(values):
        raise ValueError('配置与本场启动时不同；恢复前请使用相同配置')
    if data.get('finished'):
        raise ValueError('该场已经完成，不能重复恢复并计数')
    # 参数/恢复记录在发出任何动作前检查。调度内部损坏可重建，物理负载不可猜测。
    def invalid(key):
        raise ValueError(str(path)+'：恢复字段 '+key+' 缺失或格式不符')
    if not isinstance(data.get('run_dir'),str) or not data['run_dir']:
        invalid('run_dir')
    if not isinstance(data.get('record'),dict):
        invalid('record')
    if type(data.get('entry_completed')) is not bool:
        invalid('entry_completed')
    for key in ('initial_nav_completed','drop_arrived','task_started'):
        if key in data and type(data[key]) is not bool:
            invalid(key)
    started = data.get('started')
    if (data.get('task_started',True) and
            (type(started) not in (int,float) or not math.isfinite(started))):
        invalid('started')
    for key in ('actions','scheduler','last_put_receipt'):
        if data.get(key) is not None and not isinstance(data[key],dict):
            invalid(key)
    for key,value in data['record'].items():
        if key.endswith('_ids') or key in ('navigation_points_completed',
                'detection_points_completed','detection_points_skipped','recording_warnings'):
            if not isinstance(value,list):
                invalid('record.'+key)
        elif key in ('physical_grasps','physical_deliveries','delivery_actions_completed',
                'zone_sensor_verified_deliveries','grasp_interface_calls','place_interface_calls',
                'speech_requests','speech_done_requests'):
            if type(value) is not int or value<0:
                invalid('record.'+key)
    saved = data.get('actions') or {}
    if saved.get('load_state','empty') not in ('empty','holding','unknown'):
        invalid('actions.load_state')
    for key in ('holding','pending_pick'):
        context = saved.get(key)
        if context is None:
            continue
        if not isinstance(context,dict):
            invalid('actions.'+key)
        for field in ('object_id','point_id','attempt_id','label'):
            if not isinstance(context.get(field),str) or not context[field]:
                invalid('actions.'+key+'.'+field)
        path_value = context.get('retreat_poses',[])
        if (not isinstance(path_value,list) or any(not isinstance(pose,list) or len(pose)!=6 or
                any(type(n) not in (int,float) or not math.isfinite(n) for n in pose)
                for pose in path_value)):
            invalid('actions.'+key+'.retreat_poses')
        index = context.get('retreat_index',0)
        if type(index) is not int or not 0<=index<=len(path_value):
            invalid('actions.'+key+'.retreat_index')
    receipt = data.get('last_put_receipt')
    if receipt:
        for key in ('object_id','point_id'):
            if not isinstance(receipt.get(key),str) or not receipt[key]:
                invalid('last_put_receipt.'+key)
    return data
