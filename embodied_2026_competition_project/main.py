#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""主入口：按 jujia26 直接初始化设备后执行；运行前检查参数；--check-config仅检查不连接设备。"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time
import uuid
from core.configuration import ConfigurationRejected, load_configuration
from core.diagnostics import configuration_error
from core.faults import ParameterFault, DeviceUnavailable
from core.safety import DeadlineGate, interruptible, OperationStopped
from core.recording import fallback_directory


def main(argv=None, test_mode=False):
    # 记录启动时间用于日志；任务计时在设备初始化完成后开始。
    launched = time.monotonic()
    root = Path(__file__).resolve().parent
    local = root/'config/navigation.local.json'
    default = local if local.is_file() else root/'config/navigation.example.json'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=default)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check-config', action='store_true', help='只检查配置，不连接任何设备')
    mode.add_argument('--list-labels', action='store_true', help='查看模型类别，不要求地图点位')
    mode.add_argument('--real', action='store_true', help=argparse.SUPPRESS)  # 兼容旧命令，默认已执行。
    parser.add_argument('--match-id', help='可选场次编号；省略时自动生成')
    parser.add_argument('--resume', action='store_true', help='恢复指定或最近场次，沿用原截止时间')
    parser.add_argument('--opened-storage', action='append', default=[], metavar='FURNITURE_ID',
                        help='仅恢复：人工确认该家具已打开、把手松开并收臂，可重复填写')
    initial = parser.add_mutually_exclusive_group()
    initial.add_argument('--set-initial-pose', dest='set_initial_pose', action='store_true',
                         default=False, help='仅机器人实际摆在配置初始点时，发布该处定位')
    initial.add_argument('--no-initial-pose', dest='set_initial_pose', action='store_false',
                         help='保留当前定位（默认）；仍会导航到配置的初始点')
    parser.add_argument('--localization-confirmed', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--speech-print', action='store_true', help='仅打印语音，不会出声')
    parser.add_argument('--no-log', action='store_true', help='不保存完整日志/图像，仍保存恢复文件')
    args = parser.parse_args(argv)
    try:
        # 校验在任何设备动作之前执行；即使模板未写新阈值，也有有限默认值。
        timeout = 10
        try:
            supplied = json.loads(args.config.read_text(encoding='utf-8'))
            configured = supplied.get('timing',{}).get('config_check_timeout_s',10)
            if type(configured) in (int,float) and 0<configured<float('inf'):
                timeout = configured
        except (OSError,ValueError,AttributeError):
            pass  # 详细配置读取错误由load_configuration输出。
        check_gate = DeadlineGate()
        with check_gate.limit(time.monotonic()+timeout,timeout),interruptible(check_gate):
            config = load_configuration(args.config,root,require_poses=not args.list_labels)
    except Exception as exc:
        print(configuration_error(exc,args.config,root),file=sys.stderr)
        return 2
    if args.list_labels:
        print(json.dumps({'labels':config.labels, 'excluded_labels':config.values['excluded_labels']}, ensure_ascii=False, indent=2))
        return 0
    if args.check_config:
        print('配置与模型检查通过；未连接ROS、相机、机械臂或语音。')
        return 0
    # 显式 print 配置沿用导航演练，不隐式切换到真实机械臂。
    test_mode = test_mode or config.values['actions']['mode'] == 'print'
    if args.resume and not args.match_id:
        try:
            sources = [config.evidence_path/'latest_match.json',fallback_directory(root)/'latest_match.json']
            sources = sorted({path for path in sources if path.is_file()},key=lambda path:path.stat().st_mtime_ns,reverse=True)
            args.match_id = None
            for source in sources:
                try:
                    args.match_id = json.loads(source.read_text(encoding='utf-8'))['match_id']
                    break
                except (OSError,ValueError,KeyError):
                    continue
            if not args.match_id:
                raise ValueError('无最近场次')
        except (OSError,ValueError,KeyError):
            print('没有最近场次；请指定--match-id，或正常启动新场次。',file=sys.stderr)
            return 2
    match_id = args.match_id or ('match_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    from core.session import RobotSession, SessionRejected
    from core.workflow import Workflow
    from core.safety import OperationStopped
    print('[版本] v13：有限恢复、受阻先释放、到点直接投放、时间不足持物离场',flush=True)
    session = RobotSession(config, match_id, test_mode=test_mode,
        speech_print=args.speech_print, save_logs=not args.no_log,
        set_initial_pose=args.set_initial_pose, localization_confirmed=args.localization_confirmed,
        resume=args.resume, launched=launched)
    session.opened_storage = args.opened_storage
    try:
        with session:
            Workflow(session).run()
    except ParameterFault as exc:
        print(configuration_error(exc,config.path,root),file=sys.stderr)
        return 2
    except DeviceUnavailable as exc:
        print('设备故障暂停：'+str(exc)+'；恢复连接后使用python3 main.py --resume。',file=sys.stderr)
        return 1
    except SessionRejected as exc:
        print('无法读取运行记录：'+str(exc), file=sys.stderr)
        return 2
    except (KeyboardInterrupt, OperationStopped) as exc:
        print('流程暂停：'+str(exc)+'；排除故障后使用 python3 main.py --resume。', file=sys.stderr)
        return 1
    except Exception as exc:
        print('流程暂停：{}: {}；检查恢复文件后使用 --resume，目录：{}。'.format(
            type(exc).__name__,exc,config.evidence_path), file=sys.stderr)
        return 1
    if session.evidence.save_logs:
        if session.evidence.result_path:
            print('运行记录：'+str(session.evidence.result_path))
        else:
            print('运行记录尚未确认落盘，查看[记录提醒]；本次状态已保留内存。')
    return session.exit_code


if __name__ == '__main__':
    raise SystemExit(main())
