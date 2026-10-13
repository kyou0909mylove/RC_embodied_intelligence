# -*- coding: utf-8 -*-
"""不依赖 SDK 的目标筛选：连续多帧同一实例、腕部中心深度一致性。"""
import math
from .orientation import angle_distance


def box_iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x2-x1) * max(0, y2-y1)
    union = max(0, a[2]-a[0])*max(0, a[3]-a[1]) + max(0, b[2]-b[0])*max(0, b[3]-b[1]) - intersection
    return intersection / union if union > 0 else 0.0


def stable_candidates(frames, policy):
    """每帧一对一匹配，不能用同帧重复框凑够次数，也不把两件同名物品混成一件。

    只返回最后有效帧仍存在的候选。框稳定降低误识别，不证明没人或可安全抓取。
    wrist 的 camera_xyz_m 必须连续稳定；无深度框只能用于货架换层/重定位提示。
    """
    tracks = []
    for index, items in enumerate(frames):
        available = set(range(len(tracks)))
        for item in sorted(items, key=lambda d: d['confidence'], reverse=True):
            box = item['box_xyxy']
            if (len(box) != 4 or any(not math.isfinite(x) for x in box)
                    or box[2] <= box[0] or box[3] <= box[1]
):
                continue
            matching = [i for i in available if tracks[i]['last'] == index-1
                        and tracks[i]['item']['label'] == item['label']
                        and box_iou(tracks[i]['item']['box_xyxy'], box) >= policy['iou_threshold']]
            if matching:
                chosen = max(matching, key=lambda i: box_iou(tracks[i]['item']['box_xyxy'], box))
                available.remove(chosen)
                track = tracks[chosen]
            else:
                track = {'hits': 0, 'depths': [], 'angles':[], 'item': item, 'last': index}
                tracks.append(track)
            track['hits'] += 1
            track['last'], track['item'] = index, item
            xyz = item.get('camera_xyz_m')
            # 检测框稳定但中间深度缺失/突变，不能沿用几帧前的三维位置。
            if xyz is None or not all(math.isfinite(x) for x in xyz):
                track['depths'] = []
            else:
                track['depths'].append(list(xyz))
            if 'angle_deg' in item:
                angle = item['angle_deg']
                if angle is None or not math.isfinite(angle):
                    track['angles'] = []
                else:
                    track['angles'].append(angle)
    result = []
    for track in tracks:
        if track['last'] != len(frames)-1 or track['hits'] < policy['min_hits']:
            continue
        item = dict(track['item'], stable_hits=track['hits'])
        if 'camera_xyz_m' in item:
            depths = track['depths'][-policy['min_hits']:]
            consistent = (len(depths) >= policy['min_hits'] and
                          all(max(d[a] for d in depths)-min(d[a] for d in depths)
                              <= policy['depth_tolerance_m'] for a in range(3)))
            if not consistent:
                item['camera_xyz_m'] = None
        if 'angle_deg' in item:
            angles = track['angles'][-policy['min_hits']:]
            consistent = (len(angles)>=policy['min_hits'] and
                          all(angle_distance(a,b)<=policy['angle_tolerance_deg']
                              for a in angles for b in angles))
            if not consistent:
                item['angle_deg'] = None
        result.append(item)
    return sorted(result, key=lambda d: d['confidence'], reverse=True)
