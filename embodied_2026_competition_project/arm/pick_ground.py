# -*- coding: utf-8 -*-
"""地面调用接口：现有物品模型识别，原 catch_ty 方法执行抓取动作。"""
from types import MethodType
from core.safety import OperationStopped
from core.faults import DeviceUnavailable, ParameterFault
from .grasp import call_original


def catch_ground(controller, target, point, label):
    from .driver import load_original
    module = load_original('catch_ty')
    original = module.KinovaRobotGroud
    model = controller.cfg['profiles']['ground']['model']
    labels = model['labels']
    # 头部类别仅记录为提示；腕部独立识别原地面接口支持的全部物品。
    excluded = set(controller.session.config.values.get('excluded_labels',[]))
    requested = [name for name in labels if name not in excluded]
    if not requested:
        return {'status':'no_target','physical_action':False,'grasp_confirmed':False}
    source_detector = module.RealSenseYolo11Detector

    class ObjectDetector(source_detector):
        """只替换原垃圾目标列表，识别过程仍调用上传的识别器。"""
        def __init__(self, *args, **kwargs):
            kwargs.setdefault('conf_thres',model['confidence'])
            super().__init__(*args, **kwargs)

        def detect_targets(self, target_items, max_retry=5, show_window=True):
            result = super().detect_targets(requested,max_retry=max_retry,show_window=False)
            if result is not None:
                if controller.pending_pick is not None:
                    mapping = controller.cfg['profiles']['ground']['label_map']
                    actual = next((head for head,wrist in mapping.items() if wrist==result.name),result.name)
                    # 只更新内存里的实际类别，不在前伸和闭爪之间保存或等待反馈。
                    controller.pending_pick['label'] = actual
                    controller.pending_pick['target'].update(label=actual,wrist_label=result.name)
                reporter = getattr(controller, 'observation_reporter', None)
                if reporter is not None:
                    try:
                        reporter(point, [{'label': result.name}])
                    except (OperationStopped, DeviceUnavailable, ParameterFault):
                        raise
                    except Exception as exc:
                        controller.session.check()
                        raise DeviceUnavailable('语音', '地面播报失败，未执行后续抓取：'+str(exc)) from exc
            return result

    def invoke(kinova):
        kinova.image_to_arm = MethodType(original.image_to_arm, kinova)
        module.RealSenseYolo11Detector = ObjectDetector
        try:
            return original.catch_ground(kinova,weights_path=model['weights'])
        finally:
            module.RealSenseYolo11Detector = source_detector
    reported = dict(target or {})
    reported['head_label_hint'] = reported.get('label')
    reported['label'] = 'unknown'
    return call_original(controller, reported, point, invoke)
