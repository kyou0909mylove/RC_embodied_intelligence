# -*- coding: utf-8 -*-
"""有限阶段恢复：复核通信、取消旧动作、核对负载、回撤及收臂。

不规划新抓取路径；复用抓取记录里已有的回程。设备故障、主动停止、
比赛截止均向上传递，不能作为普通点位失败吞掉。
"""
import time
from .faults import DeviceUnavailable, ParameterFault
from .safety import ActionFailed, ExitRequested, OperationStopped, interruptible


class Recovery:
    def __init__(self, session, stage):
        self.s, self.stage = session, stage
        self.cfg = session.config.values

    def wait(self, seconds):
        end = self.s.clock() + seconds
        while self.s.clock() < end:
            self.s.check()
            time.sleep(min(.05, max(0, end-self.s.clock())))

    def probe(self, module):
        """只复核接口/新反馈，不发送移动目标；一次最多配置的诊断秒数。"""
        s = self.s
        limit = self.cfg['recovery']['communication_timeout_s']
        self.stage('CHECK_DEVICE', module=module, timeout_s=limit)
        try:
            with s.operation(limit), interruptible(s.gate):
                if module == '导航':
                    if not s.navigator.client.wait_for_server(s.rospy.Duration(limit)):
                        raise DeviceUnavailable(module, 'move_base action server 无响应')
                elif module == '机械臂':
                    s.actions.driver.check_connection(limit)
                elif module == '语音':
                    end = s.clock()+limit
                    while s.speaker.tts_pub.get_num_connections() == 0:
                        s.check()
                        if s.clock() >= end:
                            raise DeviceUnavailable(module, '/summer_tts_topic 持续无订阅')
                        self.wait(.05)
                else:
                    raise ValueError('未知设备复核模块：'+module)
        except ExitRequested:
            raise
        except DeviceUnavailable:
            raise
        except OperationStopped as exc:
            if s.gate.stopped or s.clock() >= s.match_deadline:
                raise
            raise DeviceUnavailable(module, '通信复核超时：'+str(exc)) from exc

    def arm(self, reason):
        """取消/负载恢复/收臂限次执行；恢复代码异常另走最小恢复路径。"""
        s, policy = self.s, self.cfg['recovery']
        last = None
        for attempt in range(policy['reset_retries']+1):
            self.stage('RESET_POINT', reason=reason, attempt=attempt+1,
                       timeout_s=policy['reset_timeout_s'])
            try:
                with s.operation(policy['reset_timeout_s']), interruptible(s.gate):
                    result = s.actions.reset_stalled_attempt()
                s.evidence.event('arm_recovered', reason=reason, **result)
                s.checkpoint()
                return result
            except (ExitRequested, ParameterFault):
                raise
            except Exception as exc:
                s.check()
                last = exc
                s.evidence.event('reset_retry', error_type=type(exc).__name__,
                                 error=str(exc), attempt=attempt+1)
                self.probe('机械臂')
                if attempt < policy['reset_retries']:
                    self.stage('WAIT_RESET_RETRY', seconds=policy['reset_retry_wait_s'])
                    self.wait(policy['reset_retry_wait_s'])
        # RuntimeError/KeyError等恢复软件错误不能直接被标成硬件故障。
        # 用独立、有界的最小恢复执行路径；不复用已过期的阶段计时器。
        self.stage('FALLBACK_RESET', reason=str(last))
        try:
            with s.operation(policy['reset_timeout_s']), interruptible(s.gate):
                result = s.actions.fallback_reset()
            s.evidence.event('arm_minimal_recovery', reason=str(last), **result)
            return result
        except (ExitRequested, ParameterFault):
            raise
        except Exception as exc:
            s.check()
            self.probe('机械臂')
            # 已经尝试真实的取消及home动作；没有完成这些动作就不能驾驶底盘。
            # 这是执行接口持续不完成，而不是单独一个到位容差报警。
            raise DeviceUnavailable('机械臂执行',
                '常规及最小复位均未完成，未确认收臂；最后错误：'+str(exc)) from exc

    def camera(self, vision, name, reason):
        """通信/采图故障重启后仍失败就中止；推理代码错误可让本轮换点。"""
        from camera.vision import CameraFailed
        self.stage('RESTART_CAMERA', camera=name, reason=str(reason))
        last = reason
        for attempt in range(self.cfg['recovery']['camera_restarts']+1):
            try:
                with self.s.operation(self.cfg['timing']['prepare_timeout_s']):
                    vision.restart(self.s.gate)
                return
            except ExitRequested:
                raise
            except CameraFailed as exc:
                if exc.phase=='model':
                    raise ParameterFault('model / actions.profiles.*.model','重启时模型加载失败：'+str(exc)) from exc
                self.s.check()
                last = exc
            except OperationStopped as exc:
                self.s.check()
                last = exc
        raise DeviceUnavailable(name, '相机进程重启后仍无法准备：'+str(last)) from last
