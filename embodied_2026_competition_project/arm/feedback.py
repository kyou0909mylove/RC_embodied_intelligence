# -*- coding: utf-8 -*-
"""闭合程度判持物；取消6750附近的致命死区，稳定性影响置信而不是连接判断。"""
import statistics
import time
from core.faults import DeviceUnavailable


class FeedbackUnknown(RuntimeError):
    """有反馈但释放/状态暂不能确认，由有限阶段恢复处理。"""


def classify_fingers(values, config):
    if all(v <= config['open_max_turn'] for v in values):
        return 'open'
    if all(v >= config['empty_closed_turn'] for v in values):
        return 'empty'
    blocked = sum(config['open_max_turn'] < v < config['empty_closed_turn'] for v in values)
    return 'holding' if blocked >= config['min_blocked_fingers'] else 'unknown'


class FingerFeedback:
    def __init__(self, driver, config, gate):
        self.driver,self.cfg,self.gate = driver,config,gate
        self.last_stable = False

    def _stable(self):
        end = min(self.gate.deadline,self.gate.clock()+self.cfg['timeout_s'])
        samples,last_sequence = [],None
        since = self.gate.clock()
        self.last_stable = False
        while self.gate.clock()<end:
            self.gate.check()
            sample = self.driver.finger_sample()
            if sample and sample[0]!=last_sequence and sample[2]>=since:
                last_sequence = sample[0]
                samples.append(sample)
                samples = samples[-128:]
                recent = [s for s in samples if sample[2]-s[2] <= max(.5,self.cfg['stable_span_s']*2)]
                if (len(recent)>=self.cfg['stable_samples'] and
                        recent[-1][2]-recent[0][2]>=self.cfg['stable_span_s'] and
                        all(max(s[1][a] for s in recent)-min(s[1][a] for s in recent)
                            <=self.cfg['stable_tolerance_turn'] for a in range(3))):
                    self.last_stable = True
                    return list(recent[-1][1])
            time.sleep(.02)
        self.gate.check()
        if samples and self.gate.clock()-samples[-1][2]<=self.cfg['max_age_s']:
            recent = samples[-self.cfg['stable_samples']:]
            return [statistics.median(s[1][a] for s in recent) for a in range(3)]
        raise DeviceUnavailable('机械臂三指反馈','持续没有新鲜finger_position；检查驱动及话题')

    def confirm(self, kind, attempt_id='', label='', object_id=None):
        values = self._stable()
        state = classify_fingers(values,self.cfg)
        near = state=='holding' and not any(self.cfg['open_max_turn']<v<=
            self.cfg['empty_closed_turn']-self.cfg['blocked_margin_turn'] for v in values)
        result = {'basis':'finger_position_closure','positions_turn':values,'classification':state,
                  'stable':self.last_stable,'near_boundary':near,'attempt_id':attempt_id,
                  'object_id':object_id or attempt_id,'label':label}
        if kind in ('grasp','holding'):
            result['holding'] = state in ('holding','unknown') or (state=='empty' and not self.last_stable)
            result['holding_confirmed'] = state=='holding' and self.last_stable
        elif kind=='place':
            # 只确认释放；投放之前不再等待holding读数。
            result['released'] = state=='open'
        elif kind=='initial':
            result['empty'] = state in ('open','empty')
        else:
            raise ValueError('未知夹爪确认类型：'+kind)
        if near or not self.last_stable or state=='unknown':
            print('[夹爪提醒] 读数={}，分类={}，稳定={}；按动作上下文继续'.format(
                values,state,self.last_stable),flush=True)
        return result

    def close(self):
        pass  # 订阅由driver唯一拥有。
