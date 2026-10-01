#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2026 机器人项目入口：解析参数 → 校验配置 → 实机会话 → 状态流程。

详细配置见 app/configuration.py；阶段流程见 app/workflow.py。
不加 --real 时只读文件。--real 的准备阶段会执行原开爪/回 home。
"""
import argparse
from pathlib import Path
from app.configuration import ConfigurationLoader, ConfigurationRejected


def main():
    """退出码：0 正常结束；1 运行异常/停止；2 配置或场次条件不满足。"""
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=root / "config/competition.example.json")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-config", action="store_true", help="静态校验，不导入 ROS")
    mode.add_argument("--list-tasks", action="store_true", help="显示目标清单与能力，不连接硬件")
    mode.add_argument("--real", action="store_true", help="启动实机流程，原构造器会开爪和回 home")
    parser.add_argument("--match-id", help="实机运行必填；同一局不得更换 ID 重置 600 秒")
    args = parser.parse_args()

    # 清单模式返回退出码；其他模式得到已经校验过的配置对象。
    try:
        config = ConfigurationLoader(args, root).load()
    except ConfigurationRejected:
        return 2
    if isinstance(config, int):
        return config
    if not args.real:
        print("配置结构、文件和原代码哈希检查通过；标定/模型标签/硬件尚未验证。")
        return 0

    # 延迟导入实机流程；下面模块自身也不在导入时初始化设备。
    from app.session import RobotSession, SessionRejected, SessionAborted
    from app.workflow import Workflow
    session = RobotSession(config)
    try:
        with session:
            Workflow(session.context, session).run()
    except SessionRejected:
        return 2
    except SessionAborted:
        return session.context.result_code
    return session.context.result_code


if __name__ == "__main__":
    raise SystemExit(main())
