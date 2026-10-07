# ========== 中文阅读说明 ==========
# 这是流程基础设施：统一检查截止时间与停止消息，包装已有 action 客户端。
# gate 可以理解为“发动作之前必须经过的时间与停止检查”。
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
    # 未停止且未到时返回剩余秒数；否则抛 OperationStopped。
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
        if not self.exit_suspended and now >= self.exit_deadline:
            raise ExitRequested('剩余时间不足投放及离场预算，优先离场')
        end = min(self.deadline,self.exit_deadline) if not self.exit_suspended else self.deadline
        remaining = end - now
        if self.stopped or math.isnan(remaining) or remaining <= 0:
            raise OperationStopped('停止信号或动作截止时间已到')
        return remaining

    # 【函数/方法 DeadlineGate.limit】
    # 上下文管理器：临时取整场截止、原截止、当前时间+阶段预算中的最早值。
    # 退出 with 时恢复原截止，即使块内抛异常也恢复。
    @contextmanager
    def suspend_exit(self):
        """仅离场收臂和离场本身暂时屏蔽软截止；600秒与主动停止仍生效。"""
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
        if not self.exit_suspended:
            self.deadline = min(self.deadline,self.exit_deadline)
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
    def __init__(self, client, gate, duration, succeeded=3, name=None):
        self.client, self.gate, self.duration = client, gate, duration
        self.succeeded = succeeded
        self.name = name or getattr(getattr(client, 'action_client', None), 'ns', '未知 action')

    # 【函数/方法 GuardedClient.__getattr__】
    # 本包装没有定义的属性/方法转交原客户端，例如 get_state/get_result。
    def __getattr__(self, name):
        return getattr(self.client, name)

    # 【函数/方法 GuardedClient.send_goal】
    # 发目标前检查停止与截止，并用锁保护这一小段发送过程。
    def send_goal(self, *args, **kwargs):
        # 与 stop 的停止标记互斥，避免“检查完 -> 取消 -> 又发目标”的竞态。
        with self.gate.lock:
            self.gate.check()
            return self.client.send_goal(*args, **kwargs)

    def cancel_goal(self):
        # 清理不检查截止：超时或停止后仍必须能取消在途动作。
        with self.gate.lock:
            return cancel_unfinished_goal(self.client)

    def _failure_reason(self, state):
        names = {0: 'PENDING', 1: 'ACTIVE', 2: 'PREEMPTED', 3: 'SUCCEEDED',
                 4: 'ABORTED', 5: 'REJECTED', 6: 'PREEMPTING', 7: 'RECALLING',
                 8: 'RECALLED', 9: 'LOST'}
        try:
            text = self.client.get_goal_status_text() or '驱动未提供说明'
        except Exception:
            text = '无法读取驱动说明'
        return 'action 未成功：{}，状态={}({})，驱动说明={}；停止旧动作序列'.format(
            self.name, names.get(state, 'UNKNOWN'), state, text)

    # 【函数/方法 GuardedClient.wait_for_result
    # 以不超过0.1秒的小段等待结果，同时反复检查截止和停止。
    # 拒绝0、负数、NaN或无限截止；失败结果保留状态/说明，只取消未结束的目标。
    def wait_for_result(self, timeout=None):
        try:
            remaining = self.gate.check()
            if not math.isfinite(remaining):
                raise ValueError('action 等待必须处于有限截止时间内')
            if timeout is None:
                end = self.gate.clock() + remaining
            else:
                requested = timeout.to_sec()
                if not math.isfinite(requested) or requested <= 0:
                    raise ValueError('action 等待时长必须为有限正数，禁止 Duration(0)')
                end = min(self.gate.deadline, self.gate.clock() + requested)
            while True:
                # 同时取阶段剩余时间和本次action请求剩余时间，避免一个等待超过任何一个截止。
                remaining = min(self.gate.check(), end - self.gate.clock())
                if remaining <= 0:
                    raise OperationStopped('action 等待超时')
                if self.client.wait_for_result(self.duration(min(0.1, remaining))):
                    self.gate.check()
                    state = self.client.get_state()
                    if state != self.succeeded:
                        raise ActionFailed(self.name, state, self._failure_reason(state))
                    return True
        except BaseException:
            try:
                self.cancel_goal()
            except Exception:
                pass  # 取消失败不覆盖原来的超时/停止原因。
            raise
