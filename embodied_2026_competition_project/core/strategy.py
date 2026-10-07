# -*- coding: utf-8 -*-
"""按配置顺序逐轮遍历；临时障碍只影响本轮，完整观察判空才结束复查。"""
import copy


class PointScheduler:
    def __init__(self, points, policy, clock, event, restored=None):
        self.points = [p for p in points if p.get('enabled', True)
                       and p['difficulty'] in policy['enabled_levels']]
        self.policy, self.clock, self.event = policy, clock, event
        self.next_index = 0
        self.active_point_id = None
        self.round_number = 1
        self.visited_this_round = set()
        self.states = {
            p['id']: {'visits': 0, 'empty_streak': 0, 'blocked_until': 0,
                      'disabled': False, 'permanent_reason': None, 'last_outcome': None}
            for p in self.points
        }
        self.furniture = {}
        for point in self.points:
            self.furniture.setdefault(point['furniture_id'],
                                      {'picked': {}, 'delivered': [], 'lost': []})
        if restored:
            self._restore(restored)

    def _restore(self, saved):
        """恢复顺序与本轮进度；旧版永久障碍标记不能沿用到新调度。"""
        modern = saved.get('scheduler_version') == 2
        for pid, state in saved.get('states', {}).items():
            if pid not in self.states:
                continue
            self.states[pid].update(copy.deepcopy(state))
            if not modern:
                # 旧版的 disabled 可能来自一次导航失败，旧判空也可能只看了头部。
                self.states[pid].update(disabled=False, permanent_reason=None,
                                        empty_streak=0, blocked_until=0)
        self.furniture.update(copy.deepcopy(saved.get('furniture', {})))
        ids = [point['id'] for point in self.points]
        self.round_number = max(1, int(saved.get('round_number', 1)))
        self.visited_this_round = set(saved.get('visited_this_round', [])) & set(ids)
        active, next_id = saved.get('active_point_id'), saved.get('next_point_id')
        if active in ids and active not in self.visited_this_round:
            self.active_point_id = active
            self.next_index = ids.index(active)
        elif next_id in ids:
            self.next_index = ids.index(next_id)

    def snapshot(self):
        next_id = self.points[self.next_index]['id'] if self.points else None
        return copy.deepcopy({
            'scheduler_version': 2, 'selection_mode': 'config_order_rounds',
            'round_number': self.round_number,
            'visited_this_round': sorted(self.visited_this_round),
            'next_point_id': next_id, 'active_point_id': self.active_point_id,
            'states': self.states, 'furniture': self.furniture,
        })

    def _finished(self, point):
        state = self.states[point['id']]
        visit_limit = self.policy.get('max_visits_per_point', 0)
        capped = (visit_limit > 0 and state['visits'] >= visit_limit
                  and self.active_point_id != point['id'])
        return (state['disabled'] or capped or
                state['empty_streak'] >= self.policy.get('empty_confirmations', 2))

    def pending(self):
        return [point for point in self.points if not self._finished(point)]

    def wait_seconds(self):
        """兼容旧调用：有待查点时只在轮次交界短暂等待，不再逐点冷却。"""
        return self.policy.get('round_wait_s', 3) if self.pending() else None

    def start_next_round(self):
        """只清除临时障碍；已完整判空的点与抓放库存记录继续保留。"""
        if not self.pending():
            return False
        self.round_number += 1
        self.visited_this_round.clear()
        self.next_index = 0
        self.active_point_id = None
        for state in self.states.values():
            state['blocked_until'] = 0
        self.event('search_round_started', round_number=self.round_number,
                   point_ids=[p['id'] for p in self.pending()],
                   temporary_obstacles_cleared=True)
        return True

    def next_point(self):
        """本轮每个待查点访问一次；返回 None 后由主流程刷新障碍并开始下一轮。"""
        if self.active_point_id is not None:
            return next(p for p in self.points if p['id'] == self.active_point_id)
        for offset in range(len(self.points)):
            index = (self.next_index + offset) % len(self.points)
            point = self.points[index]
            if self._finished(point) or point['id'] in self.visited_this_round:
                continue
            self.next_index = index
            self.active_point_id = point['id']
            self.states[point['id']]['visits'] += 1
            self.event('point_selected', point_id=point['id'], order_index=index + 1,
                       round_number=self.round_number, selection_mode='config_order_rounds')
            return point
        return None

    def _advance(self, point, outcome):
        if self.active_point_id != point['id']:
            return
        self.visited_this_round.add(point['id'])
        self.next_index = (self.next_index + 1) % len(self.points)
        self.active_point_id = None
        self.event('point_completed', point_id=point['id'], outcome=outcome,
                   round_number=self.round_number,
                   next_point_id=self.points[self.next_index]['id'])

    def register_pick(self, point, object_id, label):
        record = self.furniture[point['furniture_id']]
        record['picked'].setdefault(object_id, {'label': label, 'point_id': point['id']})
        # 同一家具另一观察点发现物品时，原先的判空结论需要重新核对。
        for view in self.points:
            if view['furniture_id'] == point['furniture_id']:
                self.states[view['id']]['empty_streak'] = 0

    def register_delivery(self, point, object_id, lost=False):
        ids = self.furniture[point['furniture_id']]['lost' if lost else 'delivered']
        if object_id not in ids:
            ids.append(object_id)
        self._advance(point, 'lost_in_transit' if lost else 'delivered')

    def observe(self, point, outcome, grasp_confirmed=False):
        state = self.states[point['id']]
        state['last_outcome'] = outcome
        if outcome in ('unsupported', 'storage_uncertain'):
            # 不支持的配置与中断开门状态不属于动态障碍，不能自动重放。
            state.update(disabled=True, permanent_reason=outcome)
        elif outcome == 'no_target':
            # 只有主流程确认全部配置的腕部视角均完成且没有物品，才传入 no_target。
            state['empty_streak'] += 1
        else:
            # 导航受阻、抓空、深度失败、超时等都不是家具为空的证据。
            state['empty_streak'] = 0
        state['blocked_until'] = 0
        self._advance(point, outcome)
        self.event('point_outcome', point_id=point['id'], outcome=outcome,
                   empty_rechecks=state['empty_streak'], round_number=self.round_number,
                   furniture_id=point['furniture_id'], retry_next_round=not self._finished(point))
