#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""标准库静态检查；模板的空点位单独提示，不再误报为代码检查失败。"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from core.configuration import ConfigurationRejected, load_configuration


def signatures(source):
    tree = ast.parse(source)
    result = {}
    def collect(nodes, prefix=""):
        for node in nodes:
            if isinstance(node, ast.ClassDef):
                collect(node.body, prefix + node.name + ".")
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                args = node.args
                result[prefix + node.name] = {
                    "positional": [a.arg for a in args.posonlyargs + args.args],
                    "posonly_count": len(args.posonlyargs),
                    "defaults": [ast.get_source_segment(source, d) for d in args.defaults],
                    "kwonly": [a.arg for a in args.kwonlyargs],
                    "kw_defaults": [ast.get_source_segment(source, d) if d is not None else None
                                    for d in args.kw_defaults],
                    "vararg": args.vararg.arg if args.vararg else None,
                    "kwarg": args.kwarg.arg if args.kwarg else None}
    collect(tree.body)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="可选：额外检查已填写的本地配置，不连接设备")
    args = parser.parse_args()
    errors, count = [], 0
    for path in sorted(ROOT.rglob("*.py")):
        if "runs" in path.relative_to(ROOT).parts or "__pycache__" in path.parts:
            continue
        try:
            ast.parse(path.read_text(encoding="utf-8"), filename=str(path), feature_version=8)
            count += 1
        except (OSError, SyntaxError) as exc:
            errors.append(str(exc))
    try:
        manifest = json.loads((ROOT / "SOURCE_INFO.json").read_text(encoding="utf-8"))
        for name, item in manifest["hardware_sources"].items():
            actual = signatures((ROOT / name).read_text(encoding="utf-8"))
            if actual != item["signatures"]:
                errors.append(name + "：原始硬件方法签名发生变化，请核对来源")
        load_configuration(ROOT / "config/navigation.example.json", ROOT, require_poses=False)
        weights = ROOT / "models/best.pt"
        if hashlib.sha256(weights.read_bytes()).hexdigest() != manifest["model_sha256"]:
            errors.append("best.pt 与上传模型不同")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        errors.append(str(exc))
    if args.config is not None:
        try:
            load_configuration(args.config, ROOT)
        except ConfigurationRejected as exc:
            errors.append(str(exc))
    print(json.dumps({"status": "FAIL" if errors else "PASS", "python38_syntax_files": count,
        "hardware_interfaces": "保留来源签名；未执行硬件方法",
        "template": "六个导航点待用户填写；自动初始定位另需 initial_pose",
        "device_connection": False, "errors": errors}, ensure_ascii=False, indent=2))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
