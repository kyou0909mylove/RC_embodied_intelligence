# ========== 中文阅读说明 ==========
# 这是流程基础设施：统一检查截止时间与停止消息，包装已有 action 客户端。
# gate 同时检查动作超时、急停与离场截止；到离场时间立即交回 Workflow。
# GuardedClient 不规划动作，只转发旧客户端并对发送、等待结果加限制。
# core/session.py 使用 limit 控制阶段截止，准备阶段使用 interruptible 限制旧阻塞初始化。
# 软件取消请求不能替代物理急停，注释也不代表真实设备响应已经验证。
# ==================================
"""单调时钟截止时间、有限等待、旧阻塞动作的主线程软中断。"""
from contextlib import contextmanager
import signal
import threading
import time
import math
from queue import Empty


# 【类 OperationStopped】
# 表示超时或停止的专用异常，继承 TimeoutError，方便主流程统一收尾。
class OperationStopped(TimeoutError):
    pass

class ExitRequested(OperationStopped):
    """到达离场触发时间，取消当前阶段并切换离场；不是设备故障。"""


class ActionFailed(OperationStopped):
    def __init__(self, name, state, reason):
        self.name, self.state = name, state
        super().__init__(reason)


def cancel_unfinished_goal(client):
    """只取消尚未结束的目标；读取状态失败时仍尝试停止。"""
    try:
        # ROS GoalStatus：PREEMPTED、SUCCEEDED、ABORTED、REJECTED、RECALLED。
        # LOST 不能证明驱动已停止，仍发送当前 GoalID 的取消请求。
        if client.get_state() in (2, 3, 4, 5, 8):
            return False
    except Exception:
        pass
    client.cancel_goal()
    return True


# 【类 DeadlineGate】
# 保存截止时刻和停止标志，每一步通过 check 才能继续。
class DeadlineGate:
    # 【函数/方法 DeadlineGate.__init__】
    # clock 默认单调时钟；初始无限截止供等待开始使用，不授权无限 action 等待。
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.deadline = float('inf')
        self.match_deadline = float('inf')
        self.exit_deadline = float('inf')
        self.exit_suspended = 0
        self.stopped = False
        self.lock = threading.RLock()
        self.stop_messages = None

    # 【函数/方法 DeadlineGate.check】
    # 读取最多64个停止消息，遇到 true 就锁存停止，false 不解除。
    # 返回最近截止的剩余秒数；离场截止抛 ExitRequested，急停/硬截止抛 OperationStopped。
    def check(self):
        # 主函数用队列的原生 put 作 ROS 回调；不新增机器人技能或回调函数。
        # 分批处理，避免异常消息洪流让本次时间检查无法结束。true 一旦读到即锁存停止。
        if self.stop_messages is not None:
            for _ in range(64):
                try:
                    message = self.stop_messages.get_nowait()
                except Empty:
                    break
                if bool(getattr(message, 'data', False)):
                    self.stopped = True
        now = self.clock()
        if self.stopped or now >= self.match_deadline:
            raise OperationStopped('主动停止或比赛总截止时间已到')
        exit_active = self.exit_suspended == 0
        if exit_active and now >= self.exit_deadline:
            raise ExitRequested('已到离场预留时间，停止当前任务并离场')
        end = min(self.deadline, self.match_deadline)
        if exit_active:
            end = min(end, self.exit_deadline)
        remaining = end - now
        if self.stopped or math.isnan(remaining) or remaining <= 0:
            raise OperationStopped('停止信号或动作截止时间已到')
        return remaining

    # 【函数/方法 DeadlineGate.limit】
    # 上下文管理器：临时取整场截止、原截止、当前时间+阶段预算中的最早值。
    # 退出 with 时恢复原截止，即使块内抛异常也恢复。
    @contextmanager
    def suspend_exit(self):
        """设备准备及离场收臂/导航暂时屏蔽离场截止；总截止与急停仍生效。"""
        self.exit_suspended += 1
        try:
            yield
        finally:
            self.exit_suspended -= 1

    @contextmanager
    def limit(self, deadline, timeout):
        if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
                or not math.isfinite(timeout) or timeout <= 0 or math.isnan(deadline)):
            raise ValueError('阶段截止时间和超时必须有效，超时须为有限正数')
        old = self.deadline
        self.deadline = min(old, deadline, self.clock() + timeout)
        try:
            self.check()
            yield
            self.check()
        finally:
            self.deadline = old


# 【函数/方法 interruptible】
# 提供 Linux 主线程 SIGALRM 软中断；session 准备阶段使用这个包装。
@contextmanager
def interruptible(gate):
    """Linux 主线程 SIGALRM；C 扩展可能延迟处理，不能替代驱动急停。"""
    remaining = gate.check()
    if not hasattr(signal, 'SIGALRM') or threading.current_thread() is not threading.main_thread():
        raise RuntimeError('实机阻塞动作要求 Linux 主线程 SIGALRM')
    # 【函数/方法 interruptible.alarm】
    # SIGALRM 到来时抛异常，让调用方停止当前步骤并进入清理。
    def alarm(signum, frame):
        gate.check()
        raise OperationStopped('阶段截止时间到')
    # 允许嵌套阶段保护，取旧定时器和新阶段中更早的截止，不人为延长外层预算。
    entered = time.monotonic()
    old_handler = signal.getsignal(signal.SIGALRM)
    old_timer = signal.getitimer(signal.ITIMER_REAL)
    signal.signal(signal.SIGALRM, alarm)
    duration = min(remaining,old_timer[0]) if old_timer[0] > 0 else remaining
    signal.setitimer(signal.ITIMER_REAL, duration)
    try:
        yield
        gate.check()
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old_handler)
        if old_timer[0] > 0:
            left = max(.000001,old_timer[0]-(time.monotonic()-entered))
            signal.setitimer(signal.ITIMER_REAL,left,old_timer[1])



# 【类 GuardedClient】
# 给已有 action 客户端增加门控，同时保留原 client 的其它接口。
class GuardedClient:
    """包住旧 actionlib client，每次发目标前检查，每次等待为正且有界。"""
    # 【函数/方法 GuardedClient.__init__】
    # client 为原客户端，gate 为同一门控，duration 把秒数变成 ROS Duration。
    # succeeded=3 沿用 action 的成功状态值。
    def __init__(self, client, gate, duration, succeeded=3, name=None,
                 result_timeout_s=None, log_commands=False, require_success=True):
        if result_timeout_s is not None and (
                type(result_timeout_s) not in (int, float)
                or not math.isfinite(result_timeout_s) or result_timeout_s <= 0):
            raise ValueError('action 配置等待时长必须为有限正数')
        self.client, self.gate, self.duration = client, gate, duration
        self.succeeded = succeeded
        self.name = name or getattr(getattr(client, 'action_client', None), 'ns', '未知 action')
        self.result_timeout_s, self.log_commands = result_timeout_s, log_commands
        # 导航默认仍严格；普通arm/finger按往年语义只等待返回，不以状态3门禁。
        self.require_success = require_success
        self.goal_sequence = 0
        self.successful_goal_sequence = None
        self.last_goal_result = None

    def _log(self, message):
        # 诊断输出失败不能覆盖真实的动作结果、超时或急停异常。
        if self.log_commands:
            try:
                print('[动作] {} {}'.format(self.name, message), flush=True)
            except Exception:
                pass

    def _goal_description(self, goal):
        try:
            fingers = getattr(goal, 'fingers', None)
            if fingers is not None:
                return '三指目标（驱动turn）={}'.format([
                    getattr(fingers, 'finger'+str(index)) for index in (1, 2, 3)])
            stamped = getattr(goal, 'pose', None)
            pose = getattr(stamped, 'pose', None)
            if pose is not None:
                p, q = pose.position, pose.orientation
                return 'XYZ（米）={}，四元数xyzw={}，frame={}'.format(
                    [p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
                    getattr(getattr(stamped, 'header', None), 'frame_id', ''))
        except Exception:
            pass
        return '目标已提交'

    # 【函数/方法 GuardedClient.__getattr__】
    # 本包装没有定义的属性/方法转交原客户端，例如 get_state/get_result。
    def __getattr__(self, name):
        return getattr(self.client, name)

    # 【函数/方法 GuardedClient.send_goal】
    # 发目标前检查停止与截止，并用锁保护这一小段发送过程。
    def send_goal(self, *args, **kwargs):
        # 与 stop 的停止标记互斥，避免“检查完 -> 取消 -> 又发目标”的竞态。
        with self.gate.lock:
            # 即使新发送失败，也不能沿用上一目标的成功标记。
            self.successful_goal_sequence = None
            self.last_goal_result = None
            try:
                self.gate.check()
                result = self.client.send_goal(*args, **kwargs)
            except BaseException as exc:
                self.last_goal_result = {'sequence':None, 'status':'send_failed',
                                         'state':None, 'reason':str(exc)}
                self._log('目标未能下发：{}: {}'.format(type(exc).__name__, exc))
                raise
            self.goal_sequence += 1
            self.last_goal_result = {'sequence':self.goal_sequence, 'status':'sent',
                                     'state':None, 'reason':''}
            goal = args[0] if args else kwargs.get('goal')
            self._log('目标#{}已实际下发，{}'.format(
                self.goal_sequence, self._goal_description(goal)))
            return result

    def cancel_goal(self):
        # 清理不检查截止：超时或停止后仍必须能取消在途动作。
        with self.gate.lock:
            return cancel_unfinished_goal(self.client)

    def cancel_all_goals(self):
        # 原Kinova函数超时会调用这个方法；只取消本客户端的当前机械臂目标。
        if not self.require_success:
            return self.cancel_goal()
        return self.client.cancel_all_goals()

    def _failure_reason(self, state):
        names = {0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED', 3: 'SUCCEEDED',
                 4: 'ABORTED', 5: 'REJECTED', 6: 'PREEMPTING', 7: 'RECALLING',
                 8: 'RECALLED', 9: 'LOST'}
        try:
            text = self.client.get_goal_status_text() or '驱动未提供说明'
        except Exception:
            text = '无法读取驱动说明'
        return 'action 未成功：{}，状态={}({})，驱动说明={}'.format(
            self.name, names.get(state, 'UNKNOWN'), state, text)

    # 【函数/方法 GuardedClient.wait_for_result
    # 以不超过0.1秒的小段等待结果，同时反复检查截止和停止。
    # 普通机械臂失败只记录/返回；导航、收臂、初始化和外层停止保持严格。
    def wait_for_result(self, timeout=None):
        sequence, state = self.goal_sequence, None
        try:
            remaining = self.gate.check()
            if not math.isfinite(remaining):
                raise ValueError('action 等待必须处于有限截止时间内')
            requested = None
            if timeout is not None:
                requested = timeout.to_sec()
                if not math.isfinite(requested) or requested <= 0:
                    raise ValueError('action 等待时长必须为有限正数，禁止 Duration(0)')
            # 机械臂适配层明确传配置时，以该硬限时替代原脚本写死的10/5秒。
            # 未传配置的客户端仍遵循原请求，所有等待仍服从急停/整场截止。
            effective = self.result_timeout_s if self.result_timeout_s is not None else (
                requested if requested is not None else remaining)
            end = min(self.gate.deadline, self.gate.match_deadline,
                      self.gate.clock() + effective)
            self._log('目标#{}等待上限={:.3f}秒（原请求={}秒，配置={}秒）'.format(
                sequence, max(0.0, end-self.gate.clock()), requested, self.result_timeout_s))
            while True:
                # 同时取阶段剩余时间和本次action请求剩余时间，避免一个等待超过任何一个截止。
                remaining = min(self.gate.check(), end - self.gate.clock())
                if remaining <= 0:
                    if self.require_success:
                        raise OperationStopped('action 等待超时')
                    self.successful_goal_sequence = None
                    try:
                        state = self.client.get_state()
                    except Exception:
                        pass
                    self.last_goal_result = {'sequence':sequence, 'status':'timed_out',
                                             'state':state, 'reason':'单次动作等待超时'}
                    self.cancel_goal()
                    # 外层硬截止和急停不能被普通超时返回吞掉。
                    self.gate.check()
                    self._log('目标#{}单次等待超时，状态={}；已取消当前目标，按往年顺序继续'.format(
                        sequence, state))
                    return False
                if self.client.wait_for_result(self.duration(min(0.1, remaining))):
                    self.gate.check()
                    try:
                        state = self.client.get_state()
                    except Exception:
                        if self.require_success:
                            raise
                        state = None  # 普通动作的诊断读取失败不阻止后续合/张爪。
                    if sequence != self.goal_sequence:
                        raise ActionFailed(self.name, state, '等待期间目标被新的goal覆盖，不能确认原动作成功')
                    if state != self.succeeded:
                        reason = self._failure_reason(state)
                        if self.require_success:
                            raise ActionFailed(self.name, state, reason)
                        self.successful_goal_sequence = None
                        self.last_goal_result = {'sequence':sequence, 'status':'failed',
                                                 'state':state, 'reason':reason}
                        self._log('目标#{}已返回，{}；按往年顺序继续'.format(sequence, reason))
                        return True
                    self.successful_goal_sequence = sequence
                    self.last_goal_result = {'sequence':sequence, 'status':'succeeded',
                                             'state':state, 'reason':''}
                    self._log('目标#{}成功，状态={}'.format(sequence, state))
                    return True
        except BaseException as exc:
            self.successful_goal_sequence = None
            if state is None:
                try:
                    state = self.client.get_state()
                except Exception:
                    pass
            self.last_goal_result = {'sequence':sequence, 'status':'failed',
                                     'state':state, 'reason':str(exc)}
            self._log('目标#{}未完成，状态={}，{}: {}'.format(
                sequence, state, type(exc).__name__, exc))
            try:
                self.cancel_goal()
            except Exception:
                pass  # 取消失败不覆盖原来的超时/停止原因。
            raise
