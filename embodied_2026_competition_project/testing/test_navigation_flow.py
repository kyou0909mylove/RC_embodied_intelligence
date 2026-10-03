#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直接运行本文件：真实导航，模拟相机/机械臂，默认真实语音；须先填本地配置。"""
from pathlib import Path
import sys
from datetime import datetime, timezone
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from main import main as run_main


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    readonly = any(flag in args for flag in ("--check-config", "--list-labels", "--help", "-h"))
    if not readonly:
        if "--real" not in args:
            args.append("--real")
        if not any(arg == "--match-id" or arg.startswith("--match-id=") for arg in args):
            match_id = "nav_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S") + "_" + uuid.uuid4().hex[:8]
            args.extend(["--match-id", match_id])
        if not any(flag in args for flag in ("--set-initial-pose", "--localization-confirmed")):
            args.append("--set-initial-pose")
        if "--no-log" not in args:
            args.append("--no-log")
        print("[导航测试] 真实底盘会移动；相机/机械臂模拟。默认固定初始位姿、"
              "自动测试编号，不保存流程日志。外部底盘/定位/语音节点需已启动。", flush=True)
    return run_main(args, test_mode=True)


if __name__ == "__main__":
    raise SystemExit(main())
