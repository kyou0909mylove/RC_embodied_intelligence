#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS1单点导航检查：复用工程原Navigator与SmartGoalFinder，不初始化机械臂。

--check：只连接导航服务、读取定位，默认模式。
--plan：检查到指定map坐标的规划路径，不发送移动目标。
--execute：检查路径，再发送一次导航目标；超时/停止时取消导航。
请在机器人已启动底盘、雷达、地图、定位和move_base之后使用。
"""
import argparse
import json
import math
from pathlib import Path
from queue import SimpleQueue
import signal
import sys
import time


def main():
    # 先解析参数。--help在没有ROS的电脑上也能查看。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=Path(__file__).resolve().parents[1],
                        help='完整工程目录；脚本放在该工程tools目录时无需填写')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--check', action='store_true', help='检查连接和定位，不发移动目标')
    modes.add_argument('--plan', action='store_true', help='检查规划路径，不发移动目标')
    modes.add_argument('--execute', action='store_true', help='发送一次实际导航目标')
    parser.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'),
                        help='目标的map坐标，单位米；由现场地图确定')
    parser.add_argument('--yaw-deg', type=float,
                        help='目标朝向，单位度；省略时保持检查时的底盘朝向')
    parser.add_argument('--timeout', type=float, default=30.0, help='单次导航限时秒数，默认30')
    parser.add_argument('--connect-timeout', type=float, default=20.0,
                        help='连接/定位及规划阶段各自的限时秒数，默认20')
    parser.add_argument('--stop-topic', default='/embodied/stop', help='Bool停止话题')
    parser.add_argument('--report', type=Path, help='可选JSON记录路径')
    args = parser.parse_args()
    move_mode = args.plan or args.execute
    if move_mode and args.goal is None:
        parser.error('--plan或--execute须填写--goal X Y')
    if not move_mode and (args.goal is not None or args.yaw_deg is not None):
        parser.error('--goal/--yaw-deg须与--plan或--execute一起使用')
    numbers = [args.timeout, args.connect_timeout] + list(args.goal or [])
    if args.yaw_deg is not None:
        numbers.append(args.yaw_deg)
    if not all(math.isfinite(x) for x in numbers) or not (0 < args.timeout <= 120) or not (0 < args.connect_timeout <= 60):
        parser.error('坐标/朝向须有限，导航限时须在(0,120]秒，连接限时须在(0,60]秒')

    root = args.project.expanduser().resolve()
    required = ['legacy/common/navigator2.py', 'legacy/common/position_last_second.py', 'support/safety.py']
    if any(not (root / rel).is_file() for rel in required):
        print('找不到完整工程：请把脚本放入工程tools目录，或用--project指定工程目录。', file=sys.stderr)
        return 2
    sys.path[:0] = [str(root), str(root / 'legacy/common')]
    report = {'mode': 'EXECUTE' if args.execute else 'PLAN' if args.plan else 'CHECK',
              'status': 'STARTING', 'project': str(root), 'goal_sent': False}
    navigator = stop_sub = rospy = None
    initialized = False
    old_handlers = {}
    exit_code = 1
    stage = 'IMPORTS'
    try:
        # 只导入原导航、定位和截止基础设施，完全不导入catch.py或相机模块。
        import rospy
        from actionlib_msgs.msg import GoalStatus
        from std_msgs.msg import Bool
        from navigator2 import Navigator
        from position_last_second import SmartGoalFinder
        from support.safety import DeadlineGate, GuardedClient, interruptible
        if not hasattr(signal, 'SIGALRM'):
            raise RuntimeError('该测试脚本需要机器人Linux主线程的SIGALRM支持')
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, signal.default_int_handler)
        gate = DeadlineGate(clock=time.monotonic)
        gate.stop_messages = SimpleQueue()
        gate.deadline = time.monotonic() + args.connect_timeout

        # 原构造器等待move_base与clear_costmaps服务；注入gate以使用有限等待。
        stage = 'CONNECT_AND_TF'
        with interruptible(gate):
            rospy.init_node('navigation_smoke_test', anonymous=True, disable_signals=True)
            initialized = True
            stop_sub = rospy.Subscriber(args.stop_topic, Bool, gate.stop_messages.put, queue_size=10)
            navigator = Navigator.__new__(Navigator)
            navigator.deadline_gate = gate
            Navigator.__init__(navigator, location={})
            planner = SmartGoalFinder()
            gate.check()
            if rospy.is_shutdown() or not hasattr(planner, 'make_plan_client'):
                raise RuntimeError('make_plan服务初始化失败')
            # TF监听器刚创建时需要收到数据；有限重试，不把零坐标当作定位。
            pose = None
            for _ in range(4):
                gate.check()
                pose = planner.get_robot_pose()
                if pose is not None:
                    break
                time.sleep(0.1)
            if pose is None:
                raise RuntimeError('无法读取map到base_link的TF，请检查定位与坐标系名称')
            gate.check()
            p, q = pose.pose.position, pose.pose.orientation
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                raise RuntimeError('定位包含非有限数值')
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if abs(norm - 1.0) > 0.01:
                raise RuntimeError('TF定位四元数未归一化')
            current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            report['start_pose'] = {'x': p.x, 'y': p.y, 'z': p.z, 'yaw_deg': math.degrees(current_yaw)}
            report['interfaces'] = ['move_base action', 'move_base/clear_costmaps', '/move_base/make_plan', 'map -> base_link TF']
            print('连接与定位通过：' + json.dumps(report['start_pose'], ensure_ascii=False))

        if not move_mode:
            report['status'], exit_code = 'CONNECTIVITY_OK', 0
        else:
            # 与main的NAVIGATE分支相同：先原set_goal，再原validate_goal，最后发action。
            yaw = current_yaw if args.yaw_deg is None else math.radians(args.yaw_deg)
            goal = navigator.set_goal('map', [args.goal[0], args.goal[1], 0.0],
                                      [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)])
            report['goal'] = {'frame': 'map', 'x': args.goal[0], 'y': args.goal[1], 'yaw_deg': math.degrees(yaw)}
            stage = 'PLAN'
            gate.deadline = time.monotonic() + args.connect_timeout
            with interruptible(gate):
                # 规划前重新读取起点，避免使用连接阶段较早的定位。
                pose = planner.get_robot_pose()
                if pose is None:
                    raise RuntimeError('规划前定位读取失败')
                goal.target_pose.header.stamp = rospy.Time.now()
                report['path_available'] = bool(planner.validate_goal(pose, goal.target_pose))
                gate.check()
                if not report['path_available']:
                    raise RuntimeError('make_plan未返回路径；尚未发送导航目标')
            print('路径检查通过：' + json.dumps(report['goal'], ensure_ascii=False))
            if not args.execute:
                report['status'], exit_code = 'PATH_AVAILABLE', 0
            else:
                stage = 'NAVIGATE'
                gate.deadline = time.monotonic() + args.timeout
                navigator.client = GuardedClient(navigator.client, gate, rospy.Duration,
                                                 succeeded=GoalStatus.SUCCEEDED)
                with interruptible(gate):
                    goal.target_pose.header.stamp = rospy.Time.now()
                    gate.check()
                    report['goal_sent'] = True  # 部分发送后异常，也要进入取消清理。
                    navigator.client.send_goal(goal)
                    print('已发送导航目标；Ctrl+C可取消本次测试。')
                    navigator.client.wait_for_result(rospy.Duration(gate.check()))
                    gate.check()
                    report['action_state'] = navigator.client.get_state()
                    if report['action_state'] != GoalStatus.SUCCEEDED:
                        raise RuntimeError('导航action未成功')
                report['status'], exit_code = 'NAVIGATION_SUCCEEDED', 0
    except KeyboardInterrupt:
        report.update(status='INTERRUPTED', stage=stage, error='收到Ctrl+C或SIGTERM')
    except Exception as exc:
        report.update(status='FAILED', stage=stage, error=type(exc).__name__ + ': ' + str(exc))
        if navigator is not None and hasattr(navigator, 'client'):
            try:
                report['action_state'] = navigator.client.get_state()
            except Exception as state_exc:
                report['state_read_error'] = str(state_exc)
        print('测试未通过：' + report['error'], file=sys.stderr)
    finally:
        # CHECK/PLAN不发取消请求；实际发送过目标才取消，避免只读检查干扰导航。
        if report['goal_sent'] and navigator is not None:
            try:
                navigator.stop()
            except Exception as exc:
                report['cancel_error'] = str(exc)
                exit_code = 1
        if stop_sub is not None:
            try:
                stop_sub.unregister()
            except Exception as exc:
                report['unsubscribe_error'] = str(exc)
        if initialized:
            try:
                rospy.signal_shutdown('导航测试结束')
            except Exception as exc:
                report['shutdown_error'] = str(exc)
                exit_code = 1
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
    report['exit_code'] = exit_code
    if args.report:
        try:
            path = args.report.expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        except OSError as exc:
            print('记录保存失败：' + str(exc), file=sys.stderr)
            report['report_error'] = str(exc)
            exit_code = 1
    report['exit_code'] = exit_code
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
