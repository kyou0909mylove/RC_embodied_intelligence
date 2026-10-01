#!/usr/bin/env python3
# ========== 中文阅读说明 ==========
# 这是开发检查工具，仅读源码和 JSON，不导入机器人模块、不连接硬件。
# AST 是 Python 代码的结构树；普通 # 注释不会改变这棵树。
# 它核对语法、接口签名、明确调用、本地导入、允许补丁范围和源码哈希。
# 本注释版允许“字节变化但 AST 一致”的纯注释维护，原始源文件哈希仍单独保留。
# 静态检查通过不等于真实导航、识别或抓取成功。
# ==================================
"""只读取源码/JSON并构建 AST；不导入工程模块、不运行机器人或模拟接口。"""
import argparse
import ast
import copy
import hashlib
import inspect
import json
from pathlib import Path
import sys
import builtins
from refactor_audit import audit as audit_refactor


# 【函数/方法 shape】
# 把 AST 变成可写 JSON 的标准结构，兼容不同 Python 版本的索引表示。
# 递归处理节点/列表；不执行 AST 所描述的程序。
def shape(value):
    """跨 Python 小版本规范化 AST，避免 Index/type_params 表示差异。"""
    if isinstance(value, ast.AST):
        if type(value).__name__ == 'Index':
            return shape(value.value)
        if type(value).__name__ == 'ExtSlice':
            return {'_type': 'Tuple', 'elts': shape(value.dims), 'ctx': {'_type': 'Load'}}
        result = {'_type': type(value).__name__}
        for key, item in ast.iter_fields(value):
            if key == 'type_params' and not item:
                continue
            result[key] = shape(item)
        return result
    if isinstance(value, list):
        return [shape(item) for item in value]
    return value


# 【函数/方法 digest】
# 把标准结构序列化后算 SHA256，获得用于比较的结构摘要。
def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                    separators=(',', ':')).encode('utf-8')).hexdigest()


# 【函数/方法 symbol】
# 从 Name/Attribute 提取名称，例如 kinova.arm_run；不运行这个方法。
def symbol(node):
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return symbol(node.value) + '.' + node.attr
    return ''


# 【函数/方法 declarations】
# 收集每个类/函数的参数、返回注解和装饰器，用于接口一致性比较。
def declarations(tree):
    result = {}
    # 【函数/方法 declarations.walk】
    # 内部递归遍历类的定义，给方法名加上类名前缀。
    def walk(body, prefix=''):
        for node in body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                result[prefix + node.name] = {'args': shape(node.args),
                    'returns': shape(node.returns), 'decorators': shape(node.decorator_list)}
            elif isinstance(node, ast.ClassDef):
                walk(node.body, prefix + node.name + '.')
    walk(tree.body)
    return result


# 【函数/方法 masked】
# 复制 AST，遮蔽允许维护的方法体，再比较模块其余部分。
# 这样能发现补丁是否改到了允许范围之外。
def masked(tree, methods):
    tree = copy.deepcopy(tree)
    # 【函数/方法 masked.walk】
    # 内部遍历，将允许列表中的函数体替换成空操作节点，仅影响检查用的副本。
    def walk(body, prefix=''):
        for node in body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and prefix + node.name in methods:
                node.body = [ast.Pass()]
            elif isinstance(node, ast.ClassDef):
                walk(node.body, prefix + node.name + '.')
    walk(tree.body)
    return digest(shape(tree))


# 【函数/方法 protected】
# 读取原机械臂关键位姿和手眼常量的赋值结构，单独核对这些值没有改变。
def protected(tree):
    names = {'self.homePositionMdeg', 'self.kinectA2kinova_matrix',
             'self.realsense2kinova_matrix', 'self.finger_maxTurn', 'self.finger_maxDist',
             'x_down', 'y_down', 'z_down', 'home_position', 'position_down', 'position2', 'position3'}
    result = {}
    for cls in [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'KinovaRobot']:
        for method in [n for n in cls.body if isinstance(n, ast.FunctionDef)]:
            for node in ast.walk(method):
                if isinstance(node, ast.Assign):
                    for target in node.targets:
                        label = symbol(target)
                        if label in names:
                            result.setdefault(cls.name + '.' + method.name + ':' + label, []).append(shape(node.value))
    return result


# 【函数/方法 bind_call】
# 按原函数 AST 构建 inspect.Signature，检查实参个数与关键字是否匹配。
# implicit=True 时扣除实例方法自动传入的 self；这里只绑定占位值，不调用函数。
def bind_call(function, count, keywords, implicit):
    args = function.args
    positional = list(args.posonlyargs) + list(args.args)
    parameters = []
    required = len(positional) - len(args.defaults)
    for index, arg in enumerate(positional):
        kind = inspect.Parameter.POSITIONAL_ONLY if index < len(args.posonlyargs) else inspect.Parameter.POSITIONAL_OR_KEYWORD
        default = inspect.Parameter.empty if index < required else None
        parameters.append(inspect.Parameter(arg.arg, kind, default=default))
    if args.vararg:
        parameters.append(inspect.Parameter(args.vararg.arg, inspect.Parameter.VAR_POSITIONAL))
    for arg, default in zip(args.kwonlyargs, args.kw_defaults):
        parameters.append(inspect.Parameter(arg.arg, inspect.Parameter.KEYWORD_ONLY,
                          default=inspect.Parameter.empty if default is None else None))
    if args.kwarg:
        parameters.append(inspect.Parameter(args.kwarg.arg, inspect.Parameter.VAR_KEYWORD))
    if implicit:
        parameters = parameters[1:]
    inspect.Signature(parameters).bind(*([None] * count), **{name: None for name in keywords})


# 【函数/方法 main】
# 运行静态检查器：读文件、解析、核对、汇总报告，可选另存 JSON。
# 这个 main 是检查工具入口，与 main_2026.py 的机器人入口属于不同文件。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path, help='可选 JSON 报告路径；默认只输出到终端')
    args = parser.parse_args()
    root = args.project.resolve()
    report = {'mode': 'STATIC_ONLY', 'hardware_accessed': False, 'simulation_executed': False,
              'checks': [], 'limitations': ['未验证 ROS/SDK 可用性、模型推理、标定、运动可达性或物理抓取。',
              '接口检查覆盖明确的源码调用与 commands 列表；不证明第三方库运行行为。']}
    # 【函数/方法 main.check】
    # 把单项检查的名称、PASS/FAIL和说明追加到报告，方便定位失败。
    def check(name, passed, detail):
        report['checks'].append({'check': name, 'status': 'PASS' if passed else 'FAIL', 'detail': detail})
    trees = {}
    for path in sorted(root.rglob('*.py')):
        if '__pycache__' in path.parts:
            continue
        rel = path.relative_to(root).as_posix()
        try:
            source = path.read_text(encoding='utf-8')
            trees[rel] = ast.parse(source, filename=rel, feature_version=8)
            compile(source, rel, 'exec')  # 构造代码对象，不执行、不导入。
            check('syntax:' + rel, True, 'Python 3.8 语法及当前解释器编译通过')
        except (SyntaxError, UnicodeError, ValueError) as exc:
            check('syntax:' + rel, False, str(exc))
    try:
        contracts = json.loads((root / 'validation/legacy_contracts.json').read_text(encoding='utf-8'))
        manifest = json.loads((root / 'legacy_manifest.json').read_text(encoding='utf-8'))
        delivered = {row['path']: row for row in manifest}
        checked_declarations = 0
        methods = {}
        for item in contracts['modules']:
            rel = item['path']
            tree = trees[rel]
            actual = declarations(tree)
            check('signature:' + rel, actual == item['declarations'], str(len(actual)) + ' 个原函数/方法签名')
            checked_declarations += len(actual)
            check('patch_scope:' + rel, masked(tree, item['patched_methods']) == item['unchanged_ast_sha256'],
                  '仅允许列表中的已有方法体改变')
            check('geometry:' + rel, protected(tree) == item['protected_assignments'], '保留原位姿、运动学及手眼数值')
            check('hash:' + rel, hashlib.sha256((root / rel).read_bytes()).hexdigest() == delivered[rel]['sha256'],
                  '当前文件与交付清单一致')
            if not item['patched_methods']:
                # 加注释会改变文件字节；注释版改查完整AST，仍使用原始压缩包的结构基线。
                # 未标记为注释版的交付继续要求逐字节保持，原始哈希不被当前哈希覆盖。
                if delivered[rel].get('comments_only', False):
                    check('original_ast:' + rel, masked(tree, []) == item['unchanged_ast_sha256'],
                          '中文注释维护，原始可执行AST保持')
                else:
                    check('original_bytes:' + rel, delivered[rel]['sha256'] == item['original_sha256'], '原始压缩包逐字节保持')
            check('function_inventory:' + rel,
                  sorted(digest(shape(n.args)) + ':' + n.name for n in ast.walk(tree)
                         if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))) == item['all_function_shapes'],
                  '没有新增/删除或修改函数签名，包括嵌套函数')
            for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
                for method in [n for n in cls.body if isinstance(n, ast.FunctionDef)]:
                    methods[cls.name + '.' + method.name] = method
            for node in [n for n in tree.body if isinstance(n, ast.FunctionDef)]:
                methods[node.name] = node
        report['legacy_modules_checked'] = len(contracts['modules'])
        report['legacy_declarations_checked'] = checked_declarations
        local_modules = {'support.safety': 'support/safety.py',
                         'refactor_audit': 'tools/refactor_audit.py'}
        for rel in trees:
            if rel.startswith('app/'):
                module = rel[:-3].replace('/', '.')
                if module.endswith('.__init__'):
                    module = module[:-9]
                local_modules[module] = rel
        for rel in delivered:
            if rel.startswith('legacy/common/'):
                local_modules[Path(rel).stem] = rel
            elif rel == 'legacy/tongyong_25/base_controller.py':
                local_modules['base_controller'] = rel
            else:
                local_modules[rel[len('legacy/tongyong_25/'):].replace('/', '.')[:-3]] = rel
        local_imports = 0
        for rel, tree in trees.items():
            for node in ast.walk(tree):
                if not isinstance(node, ast.ImportFrom):
                    continue
                module = node.module or ''
                if node.level:
                    parents = list(Path(rel).parent.parts)
                    module = '.'.join(parents[:len(parents) - node.level + 1]
                                      + ([module] if module else []))
                if module in local_modules:
                    source_tree = trees[local_modules[module]]
                    exports = {n.name for n in source_tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))}
                    exports.update(n.targets[0].id for n in source_tree.body
                                   if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name))
                    exports.update(n.target.id for n in source_tree.body
                                   if isinstance(n, ast.AnnAssign) and isinstance(n.target, ast.Name))
                    missing = [alias.name for alias in node.names if alias.name not in exports]
                    check('local_import:' + rel + ':L' + str(node.lineno), not missing,
                          module + ' -> ' + ', '.join(alias.name for alias in node.names))
                    local_imports += 1
        report['local_import_statements_checked'] = local_imports
        source = json.loads((root / 'support/SOURCE.json').read_text(encoding='utf-8'))
        check('support_source', hashlib.sha256((root / source['local_path']).read_bytes()).hexdigest() == source['local_sha256'],
              '复用门控模块与记录版本一致')
        main_tree = trees['main_2026.py']
        functions = [n.name for n in ast.walk(main_tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]
        check('main_function_boundary', functions == ['main'] and not any(isinstance(n, (ast.ClassDef, ast.Lambda))
              for n in ast.walk(main_tree)), '入口仅 main()；辅助逻辑移入 app，机器人技能继续复用旧接口')
        check('main_size', len((root / 'main_2026.py').read_text(encoding='utf-8').splitlines()) <= 100,
              '主入口不超过 100 行')
        init = methods['KinovaRobot.__init__']
        guarded = {symbol(n.targets[0]): n.lineno for n in ast.walk(init)
                   if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(n.value, ast.Call)
                   and symbol(n.value.func) == 'GuardedClient'}
        motion_lines = [n.lineno for n in ast.walk(init) if isinstance(n, ast.Call)
                        and symbol(n.func) in ('self.arm_run', 'self.finger_run')]
        check('kinova_constructor_guards', all(name in guarded and guarded[name] < min(motion_lines)
              for name in ('self.client_arm', 'self.client_finger')), '两类 action client 在原构造器首次运动前包装')
        session_tree = trees['app/session.py']
        init_calls = [n for n in ast.walk(session_tree) if isinstance(n, ast.Call)
                      and symbol(n.func) == 'KinovaRobot.__init__']
        injected = [n.lineno for n in ast.walk(session_tree) if isinstance(n, ast.Assign)
                    and any(symbol(t) == 'self.kinova.deadline_gate' for t in n.targets)
                    and symbol(n.value) == 'self.gate']
        check('kinova_gate_injection', bool(injected) and all(min(injected) < n.lineno for n in init_calls),
              '调用原签名构造器前注入同一门控对象')
        receivers = {'navigator': 'Navigator', 'base': 'Base', 'planner': 'SmartGoalFinder',
                     'converter': 'CoordinateConverter', 'detector': 'ItemsDetector', 'camera': 'KinectCamera',
                     'kinova': 'KinovaRobot', 'ground_detector': 'RealSenseYolo11Detector',
                     'speaker': 'SummerTTSSpeaker', 'check': 'RealSenseYolo11DetectorDesk'}
        classes = set(receivers.values())
        calls = 0
        production = {rel: tree for rel, tree in trees.items()
                      if rel == 'main_2026.py' or rel.startswith('app/')}
        # 扫描所有新流程模块，不再只检查旧主文件；保留原调用参数绑定规则。
        call_nodes = [(rel, node) for rel, tree in production.items() for node in ast.walk(tree)]
        for rel, node in call_nodes:
            name, implicit, count, keywords = '', False, 0, []
            if isinstance(node, ast.Call):
                called = symbol(node.func)
                if '.' in called:
                    receiver, method = called.rsplit('.', 1)
                    short_receiver = receiver.rsplit('.', 1)[-1]
                    if short_receiver in receivers:
                        name, implicit = receivers[short_receiver] + '.' + method, True
                    elif receiver in classes and method == '__init__':
                        name = called
                elif called in classes:
                    name, implicit = called + '.__init__', True
                elif called in methods:
                    name = called
                if not name:
                    continue
                if any(isinstance(arg, ast.Starred) for arg in node.args) or any(k.arg is None for k in node.keywords):
                    check('call:' + rel + ':L' + str(node.lineno), False, '无法静态展开参数: ' + name)
                    continue
                count, keywords = len(node.args), [k.arg for k in node.keywords]
            elif (isinstance(node, ast.Tuple) and len(node.elts) == 2 and isinstance(node.elts[1], ast.Dict)
                  and symbol(node.elts[0]).rsplit('.', 1)[0].rsplit('.', 1)[-1] == 'kinova'):
                name, implicit = 'KinovaRobot.' + symbol(node.elts[0]).split('.')[-1], True
                keywords = [key.value for key in node.elts[1].keys if isinstance(key, ast.Constant) and isinstance(key.value, str)]
            else:
                continue
            calls += 1
            try:
                bind_call(methods[name], count, keywords, implicit)
                check('call:' + rel + ':L' + str(node.lineno), True, name)
            except (KeyError, TypeError, ValueError) as exc:
                check('call:' + rel + ':L' + str(node.lineno), False, name + ': ' + str(exc))
        report['explicit_legacy_call_sites_checked'] = calls
        cleanup = [n for n in ast.walk(session_tree) if isinstance(n, ast.FunctionDef)
                   and n.name in ('finish', '_stop_resources', '_shutdown', '__exit__')]
        cleanup += [n for n in ast.walk(trees['app/evidence.py']) if isinstance(n, ast.FunctionDef)
                    and n.name == 'finalize']
        motion = {'goto_home', 'goto_detect', 'arm_run', 'finger_run', 'close_finger', 'open_finger',
                  'catch_table', 'catch_table_short', 'catch_ground', 'send_goal'}
        cleanup_motion = [symbol(n.func) for s in cleanup for n in ast.walk(s)
                          if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr in motion]
        check('cleanup_no_motion', not cleanup_motion, '清理路径没有回 home、松爪或发送新目标')
        check('static_cli_boundary', all('run_scenario' not in (root / rel).read_text(encoding='utf-8')
                                        for rel in production),
              '当前交付不通过模拟替身生成完成结论')
        # 只检查模块顶层导入；设备/模型模块必须留在 prepare 等实机方法中。
        hardware_modules = ('rospy', 'numpy', 'cv2', 'actionlib', 'kinova_msgs', 'pykinect_azure',
                            'pyrealsense2', 'ultralytics', 'navigator2', 'base_controller', 'catch_ground')
        for rel, tree in production.items():
            early_imports = []
            for node in tree.body:
                if isinstance(node, ast.Import):
                    early_imports.extend(alias.name for alias in node.names)
                elif isinstance(node, ast.ImportFrom):
                    early_imports.append(node.module or '')
            forbidden = [m for m in early_imports if m.split('.')[0] in hardware_modules]
            check('hardware_import_boundary:' + rel, not forbidden,
                  '模块顶层仅标准库/流程模块，不导入 ROS、模型或设备驱动')
            # 保守名称绑定检查：检查方法名称是否来自参数、赋值、导入、模块或内置。
            global_names = set(dir(builtins)) | {'__file__', '__name__', '__doc__'}
            for node in tree.body:
                if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
                    global_names.add(node.name)
                elif isinstance(node, ast.Import):
                    global_names.update(alias.asname or alias.name.split('.')[0] for alias in node.names)
                elif isinstance(node, ast.ImportFrom):
                    global_names.update(alias.asname or alias.name for alias in node.names)
                elif isinstance(node, (ast.Assign, ast.AnnAssign)):
                    global_names.update(n.id for n in ast.walk(node)
                                        if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Store))
            unknown = []
            for fn in [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]:
                bound = global_names | {a.arg for a in list(fn.args.posonlyargs) + list(fn.args.args)
                                        + list(fn.args.kwonlyargs)}
                if fn.args.vararg:
                    bound.add(fn.args.vararg.arg)
                if fn.args.kwarg:
                    bound.add(fn.args.kwarg.arg)
                for node in ast.walk(fn):
                    if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
                        bound.add(node.id)
                    elif isinstance(node, ast.Import):
                        bound.update(a.asname or a.name.split('.')[0] for a in node.names)
                    elif isinstance(node, ast.ImportFrom):
                        bound.update(a.asname or a.name for a in node.names)
                    elif isinstance(node, ast.ExceptHandler) and node.name:
                        bound.add(node.name)
                unknown += [(fn.name, n.id, n.lineno) for n in ast.walk(fn)
                            if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load) and n.id not in bound]
            check('name_bindings:' + rel, not unknown, '名称绑定检查：' + str(unknown))
        context_class = next(n for n in trees['app/context.py'].body
                             if isinstance(n, ast.ClassDef) and n.name == 'RunContext')
        context_fields = {n.target.id for n in context_class.body if isinstance(n, ast.AnnAssign)}
        context_fields.update(n.name for n in context_class.body if isinstance(n, ast.FunctionDef))
        missing_fields = [(rel, n.attr) for rel, tree in production.items() for n in ast.walk(tree)
                          if isinstance(n, ast.Attribute) and isinstance(n.value, ast.Name)
                          and n.value.id == 'ctx' and n.attr not in context_fields]
        check('context_fields', not missing_fields, '运行上下文读写字段均有定义：' + str(missing_fields))
        for name, passed, detail in audit_refactor(root, trees):
            check(name, passed, detail)
        report['production_modules_checked'] = len(production)
        cfg = json.loads((root / 'config/competition.example.json').read_text(encoding='utf-8'))
        check('config_schema', cfg['schema_version'] == 3, '保留 schema 3')
        catalog = cfg['task_catalog']
        check('catalog_template', len(catalog) == 18 and all(t['status'] == 'unsupported' for t in catalog if t['level'] == 3),
              '18 条模板，三级能力未启用')
        report['deployment_configuration_complete'] = False  # 模板刻意保留现场空值。
    except (KeyError, OSError, ValueError, StopIteration) as exc:
        check('project_structure', False, str(exc))
    failures = sum(item['status'] == 'FAIL' for item in report['checks'])
    report['status'] = 'PASS' if not failures else 'FAIL'
    report['checks_passed'] = len(report['checks']) - failures
    report['checks_failed'] = failures
    text = json.dumps(report, ensure_ascii=False, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding='utf-8')
    print(text, end='')
    return 1 if failures else 0


# 【脚本入口】只有直接运行本文件才执行这里；从其它文件import不会进入这个分支。
# 这里调用本文件自己的main；该文件的用途见开头中文说明。
if __name__ == '__main__':
    sys.exit(main())
