# -*- coding: utf-8 -*-
"""运行证据写入。只处理记录与文件，所有写入仍受原阶段截止约束。"""
import json
import sys
import time
from datetime import datetime, timezone


class EvidenceWriter:
    """持有运行目录；不建立后台线程，不延后交付确认。"""

    def __init__(self, run_dir):
        """目录已由设备会话创建；保存该目录供扫描和日志使用。"""
        self.run_dir = run_dir

    def checkpoint(self, context, gate, slots):
        """continue、异常后的检查点沿用原截止判据，不支持续赛恢复。"""
        ctx = context
        run_dir = self.run_dir
        if time.monotonic() < ctx.operation_deadline and not gate.stopped:
            gate.check()
            checkpoint_time = datetime.now(timezone.utc).isoformat()
            ctx.record.update(checkpoint=True, checkpoint_wall_utc=checkpoint_time,
                          last_state=ctx.state, holding_at_stop=ctx.holding, slots=slots,
                          elapsed_s=round(time.monotonic() - ctx.started, 3))
            checkpoint_path = run_dir / "match_record.tmp"
            checkpoint_path.write_text(json.dumps(ctx.record, ensure_ascii=False, indent=2), encoding="utf-8")
            checkpoint_path.replace(run_dir / "match_record.json")
            with (run_dir / "events.jsonl").open("a", encoding="utf-8") as event_file:
                for event in ctx.record["events"][ctx.journal_cursor:]:
                    event_file.write(json.dumps(dict(event, logged_wall_utc=checkpoint_time,
                        logged_elapsed_s=ctx.record["elapsed_s"]), ensure_ascii=False) + "\n")
            ctx.journal_cursor = len(ctx.record["events"])
            gate.check()

    def finalize(self, context, slots):
        """保存最终台账；写入失败设置退出码 1，不新增机器人动作。"""
        ctx = context
        run_dir = self.run_dir
        ctx.record.update(checkpoint=False, last_state=ctx.state, holding_at_stop=ctx.holding, slots=slots,
                      elapsed_s=None if ctx.started is None else round(time.monotonic() - ctx.started, 3),
                      return_code=ctx.result_code)
        if ctx.task is not None and ctx.attempt is not None and ctx.attempt["status"] in ("GRASP_STARTED", "HOLD_INDICATED"):
            ctx.attempt["status"] = "INTERRUPTED_WITH_POSSIBLE_OBJECT"
        try:
            tmp_record = run_dir / "match_record.tmp"
            tmp_record.write_text(json.dumps(ctx.record, ensure_ascii=False, indent=2), encoding="utf-8")
            tmp_record.replace(run_dir / "match_record.json")
            with (run_dir / "events.jsonl").open("a", encoding="utf-8") as event_file:
                for event in ctx.record["events"][ctx.journal_cursor:]:
                    event_file.write(json.dumps(dict(event, logged_wall_utc=datetime.now(timezone.utc).isoformat(),
                        logged_elapsed_s=ctx.record["elapsed_s"]), ensure_ascii=False) + "\n")
        except OSError as exc:
            print("运行记录写入失败：" + str(exc), file=sys.stderr)
            ctx.result_code = 1
