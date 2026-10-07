# -*- coding: utf-8 -*-
"""新增时间默认值集中于此；现场覆盖统一填写config/navigation.local.json。

现有local值优先，地图、手眼矩阵、home/place等物理参数不在此修改。
docs/TIMEOUTS.md列出每个消费者及超时后的处理。
"""
import math

DEFAULTS = {
    'timing': {'config_check_timeout_s':10, 'state_timeout_s':5,
               'record_timeout_s':2, 'cleanup_timeout_s':3},
    'recovery': {'communication_timeout_s':10, 'nav_server_timeout_s':5,
                 'drop_max_attempts':3, 'reset_retries':1, 'reset_retry_wait_s':2,
                 'exit_retry_wait_s':5},
    'actions': {'arm_action_timeout_s':20, 'finger_action_timeout_s':10,
                'server_timeout_s':5, 'initial_feedback_timeout_s':10, 'cancel_timeout_s':3, 'home_timeout_s':20,
                'startup_open_timeout_s':10, 'startup_home_timeout_s':20,
                'abandon_timeout_s':30, 'reach_estimate_s':5,
                'return_estimate_s':5, 'close_estimate_s':5, 'observe_retries':1},
    'speech': {'subscriber_timeout_s':5, 'max_timeout_s':60, 'completion_slack_s':2},
    'camera': {'process_join_timeout_s':.2},
    'localization': {'publish_interval_s':.5},
}


def apply_defaults(config):
    for section, values in DEFAULTS.items():
        group = config.get(section)
        if isinstance(group, dict):
            for key, value in values.items():
                group.setdefault(key, value)
    wrist = config.get('actions', {}).get('wrist', {}) if isinstance(config.get('actions'),dict) else {}
    if isinstance(wrist,dict):
        wrist.setdefault('frame_timeout_s',1)
    return config


def validate(config, errors):
    for section, values in DEFAULTS.items():
        group = config.get(section, {})
        for key in values:
            value = group.get(key)
            if key in ('drop_max_attempts','reset_retries','observe_retries'):
                minimum = 1 if key == 'drop_max_attempts' else 0
                valid = type(value) is int and value >= minimum
            else:
                valid = type(value) in (int,float) and math.isfinite(value) and value > 0
            if not valid:
                errors.append(section+'.'+key+' 须为范围内有限正数/整数')
    value = config['actions']['wrist'].get('frame_timeout_s')
    if type(value) not in (int,float) or not math.isfinite(value) or value <= 0:
        errors.append('actions.wrist.frame_timeout_s 须为有限正秒数')
