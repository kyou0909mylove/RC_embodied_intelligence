#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""唯一主入口；testing/test_navigation_flow.py 复用此入口和同一套导航流程。"""
import argparse
import json
from pathlib import Path
import sys
from core.configuration import ConfigurationRejected, load_configuration


def main(argv=None, test_mode=False):
    root = Path(__file__).resolve().parent
    default = root / "config/navigation.local.json"
    if not default.is_file():
        default = root / "config/navigation.example.json"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=default)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-config", action="store_true", help="只校验点位和模型，不连接设备")
    mode.add_argument("--list-labels", action="store_true", help="查看上传模型的标签，不要求点位填写")
    mode.add_argument("--real", action="store_true", help="发送真实 move_base 导航目标")
    parser.add_argument("--match-id", help="本次运行唯一编号，不用于生成或改变点位")
    localization = parser.add_mutually_exclusive_group()
    localization.add_argument("--set-initial-pose", action="store_true",
                              help="机器人已在固定起始位置；复用 jujia26 初始位姿方法，随后人工确认定位")
    localization.add_argument("--localization-confirmed", action="store_true",
                              help="已在 RViz 人工确认当前地图、姿态及激光对齐；不重新发布初始位姿")
    parser.add_argument("--speech-print", action="store_true", help="语音也只打印；不会真的出声")
    parser.add_argument("--no-log", action="store_true", help="不保存流程日志/图像；保留少量场次锁和启动标记")
    args = parser.parse_args(argv)
    try:
        config = load_configuration(args.config, root, require_poses=not args.list_labels)
        if args.set_initial_pose and config.values.get("initial_pose") is None:
            raise ConfigurationRejected("--set-initial-pose 需要单独填写 initial_pose，不能默认用入场点代替")
    except ConfigurationRejected as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if args.list_labels:
        print(json.dumps({"model": str(config.model_path), "labels": config.labels}, ensure_ascii=False, indent=2))
        return 0
    if not args.real:
        print("配置与模型检查通过；未初始化 ROS、相机、机械臂或语音。")
        print("顺序：初始化/定位确认 → 入场 → " +
              " → ".join(p["name"] + "/检测/语音/抓取占位/投放/放下占位"
                         for p in config.values["detection_points"]) + " → 离场")
        print("相机模式：" + ("模拟" if test_mode else "真实；机械臂仍是 print 接口"))
        return 0
    if test_mode:
        # 独立导航演练不等待未知的硬件开门信号；只改内存，不覆盖本地配置。
        config.values["start"]["mode"] = "immediate"
    elif config.values["start"]["mode"] != "topic":
        print("比赛主入口须使用 start.mode=topic，并将已有开始话题与裁判开门信号同步；"
              "不能以初始化后的固定延迟代替正式比赛起点。导航演练请运行 testing/test_navigation_flow.py。",
              file=sys.stderr)
        return 2
    if not test_mode and (args.no_log or not config.values["recording"]["save_images"]):
        print("[识别证据] 当前未保存识别图片；规则争议处理要求带时间戳、框和名称的图片。"
              "建议启用 recording.save_images 并省略 --no-log。", flush=True)
    from core.session import RobotSession, SessionRejected
    from core.workflow import Workflow
    from core.safety import OperationStopped
    session = RobotSession(config, args.match_id, test_mode=test_mode,
                           speech_print=args.speech_print, save_logs=not args.no_log,
                           set_initial_pose=args.set_initial_pose,
                           localization_confirmed=args.localization_confirmed)
    try:
        with session:
            Workflow(session).run()
    except SessionRejected as exc:
        print("启动条件不满足：" + str(exc), file=sys.stderr)
        return 2
    except (KeyboardInterrupt, OperationStopped) as exc:
        print("流程停止：" + str(exc), file=sys.stderr)
        return 1
    except Exception as exc:
        print("流程失败：{}: {}".format(type(exc).__name__, exc), file=sys.stderr)
        return 1
    if session.evidence.save_logs:
        print("运行记录：" + str(session.evidence.run_dir / "result.json"))
    else:
        print("未保存流程日志或图像；仅保留场次启动标记。")
    return session.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
