# -*- coding: utf-8 -*-
"""按点位顺序循环；确认抓取扣减物品数，剩余为零后退出搜索队列。"""
import copy


class PointScheduler:
    def __init__(self, points, policy, clock, event, restored=None, confirmed_object_ids=()):
        self.points = [p for p in points if p.get('enabled', True)
                       and p['difficulty'] in policy['enabled_levels']]
        self.policy, self.clock, self.event = policy, clock, event
        self.next_index = 0
        self.active_point_id = None
        self.round_number = 1
        self.visited_this_round = set()
        self.states = {
            p['id']: {'visits': 0, 'remaining_count': p.get('object_count',1),
                      'counted_object_ids': [], 'last_outcome': None}
            for p in self.points
        }
        if any(type(state['remaining_count']) is not int or state['remaining_count']<0
               for state in self.states.values()):
            raise ValueError('detection_points.*.object_count 须为非负整数')
        self.furniture = {}
        for point in self.points:
            self.furniture.setdefault(point['furniture_id'],
                                      {'picked': {}, 'delivered': [], 'lost': []})
        if restored:
            self._restore(restored,confirmed_object_ids)

    def _restore(self, saved, confirmed_object_ids=()):
        """恢复剩余数量；旧快照按真实确认的抓取ID迁移，不把疑似负载当成功。"""
        modern = saved.get('scheduler_version') == 3
        self.furniture.update(copy.deepcopy(saved.get('furniture', {})))
        confirmed = set(confirmed_object_ids)
        for pid, state in saved.get('states', {}).items():
            if pid not in self.states:
                continue
            current = self.states[pid]
            current.update(visits=state.get('visits',0),last_outcome=state.get('last_outcome'))
            if modern:
                current['remaining_count'] = state['remaining_count']
                current['counted_object_ids'] = list(state.get('counted_object_ids',[]))
                if type(current['remaining_count']) is not int or current['remaining_count']<0:
                    raise ValueError('恢复快照 remaining_count 须为非负整数')
            else:
                picked = [oid for record in self.furniture.values()
                          for oid,item in record.get('picked',{}).items()
                          if item.get('point_id')==pid and oid in confirmed]
                current['counted_object_ids'] = list(dict.fromkeys(picked))
                current['remaining_count'] = max(0,current['remaining_count']-len(picked))
        ids = [point['id'] for point in self.points]
        self.round_number = max(1, int(saved.get('round_number', 1)))
        self.visited_this_round = set(saved.get('visited_this_round', [])) & set(ids)
        active, next_id = saved.get('active_point_id'), saved.get('next_point_id')
        if (active in ids and active not in self.visited_this_round
                and self.states[active]['remaining_count']>0):
            self.active_point_id = active
            self.next_index = ids.index(active)
        elif next_id in ids:
            self.next_index = ids.index(next_id)

    def snapshot(self):
        next_id = self.points[self.next_index]['id'] if self.points else None
        return copy.deepcopy({
            'scheduler_version': 3, 'selection_mode': 'config_order_counts',
            'round_number': self.round_number,
            'visited_this_round': sorted(self.visited_this_round),
            'next_point_id': next_id, 'active_point_id': self.active_point_id,
            'states': self.states, 'furniture': self.furniture,
        })

    def _finished(self, point):
        return self.states[point['id']]['remaining_count']<=0

    def pending(self):
        return [point for point in self.points if not self._finished(point)]

    def wait_seconds(self):
        """兼容旧调用：有待查点时只在轮次交界短暂等待，不再逐点冷却。"""
        return self.policy.get('round_wait_s', 3) if self.pending() else None

    def start_next_round(self):
        """按剩余点位开始下一轮；失败不扣数量，完成的点不再访问。"""
        if not self.pending():
            return False
        self.round_number += 1
        self.visited_this_round.clear()
        self.next_index = 0
        self.active_point_id = None
        self.event('search_round_started', round_number=self.round_number,
                   point_ids=[p['id'] for p in self.pending()],
                   temporary_obstacles_cleared=True)
        return True

    def next_point(self):
        """本轮每个待查点访问一次；返回 None 后由主流程刷新障碍并开始下一轮。"""
        if self.active_point_id is not None:
            active = next(p for p in self.points if p['id'] == self.active_point_id)
            if not self._finished(active):
                return active
            self._advance(active,'object_count_completed')
        for offset in range(len(self.points)):
            index = (self.next_index + offset) % len(self.points)
            point = self.points[index]
            if self._finished(point) or point['id'] in self.visited_this_round:
                continue
            self.next_index = index
            self.active_point_id = point['id']
            self.states[point['id']]['visits'] += 1
            self.event('point_selected', point_id=point['id'], order_index=index + 1,
                       round_number=self.round_number, selection_mode='config_order_counts',
                       remaining_count=self.states[point['id']]['remaining_count'])
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

    def register_pick(self, point, object_id, label, grasp_confirmed=True):
        record = self.furniture[point['furniture_id']]
        item = record['picked'].setdefault(object_id, {'label':label,'point_id':point['id']})
        item['grasp_confirmed'] = bool(item.get('grasp_confirmed') or grasp_confirmed)
        state = self.states[point['id']]
        if grasp_confirmed and object_id not in state['counted_object_ids']:
            state['counted_object_ids'].append(object_id)
            state['remaining_count'] = max(0,state['remaining_count']-1)
            self.event('point_object_count_changed',point_id=point['id'],object_id=object_id,
                       remaining_count=state['remaining_count'],removed_from_search=self._finished(point))

    def register_delivery(self, point, object_id, lost=False):
        ids = self.furniture[point['furniture_id']]['lost' if lost else 'delivered']
        if object_id not in ids:
            ids.append(object_id)
        self._advance(point, 'lost_in_transit' if lost else 'delivered')

    def observe(self, point, outcome, grasp_confirmed=False):
        state = self.states[point['id']]
        state['last_outcome'] = outcome
        # 数量只在 register_pick 收到确认抓取时扣减；空照/导航误差都保留该点。
        self._advance(point, outcome)
        self.event('point_outcome', point_id=point['id'], outcome=outcome,
                   remaining_count=state['remaining_count'], round_number=self.round_number,
                   furniture_id=point['furniture_id'], retry_next_round=not self._finished(point))
