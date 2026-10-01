# -*- coding: utf-8 -*-
"""重构迁移的 AST 核对工具：读取源码，不导入或执行机器人模块。"""
import ast
import copy
import hashlib
import json
import textwrap


def symbol(node):
    """取得变量/属性路径，用于把 ctx.task 等还原为迁移前局部变量。"""
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        return symbol(node.value) + '.' + node.attr
    return ''


def shape(value):
    """忽略行号以及 Python 小版本的空结构字段，保留实际语句结构。"""
    if isinstance(value, ast.AST):
        if type(value).__name__ == 'Index':
            return shape(value.value)
        return {'type': type(value).__name__, **{
            key: shape(item) for key, item in ast.iter_fields(value)
            if not (key == 'type_params' and not item)}}
    if isinstance(value, list):
        return [shape(item) for item in value]
    return value


def digest(value):
    """生成源码结构摘要，用于比较迁移前后的条件和调用顺序。"""
    return hashlib.sha256(json.dumps(shape(value), sort_keys=True, ensure_ascii=False,
                                    separators=(',', ':')).encode('utf-8')).hexdigest()


class RestoreNames(ast.NodeTransformer):
    """仅还原已记录的变量搬移与控制出口，不放宽业务条件比较。"""

    def __init__(self, mapping, outer_continue=False):
        """保存反向名称映射；False 返回只在阶段方法中对应原 continue。"""
        self.mapping = mapping
        self.outer_continue = outer_continue

    def visit_Attribute(self, node):
        """ctx/self 属性还原为原名称，保留 Load/Store 上下文。"""
        name = symbol(node)
        if name in self.mapping:
            return ast.copy_location(ast.Name(id=self.mapping[name], ctx=node.ctx), node)
        return self.generic_visit(node)

    def visit_Return(self, node):
        """原 continue 迁入方法后使用 return False，中间循环 continue 保持。"""
        if self.outer_continue and isinstance(node.value, ast.Constant) and node.value.value is False:
            return ast.copy_location(ast.Continue(), node)
        return self.generic_visit(node)

    def visit_Raise(self, node):
        """配置/场次拒绝异常对应原已打印错误后的 return 2。"""
        if isinstance(node.exc, ast.Call) and symbol(node.exc.func) in (
                'ConfigurationRejected', 'SessionRejected'):
            return ast.copy_location(ast.Return(value=ast.Constant(value=2)), node)
        return self.generic_visit(node)


def find_method(tree, name):
    """在当前模块中找唯一同名方法；不运行该类。"""
    found = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name]
    if len(found) != 1:
        raise ValueError('无法唯一定位方法：' + name)
    return found[0]


def normalize(body, row):
    """去掉明确新增的别名和返回值，然后恢复原变量表示。"""
    body = copy.deepcopy(body)
    if row.get('complete_return'):
        last = body.pop()
        if not (isinstance(last, ast.Return) and isinstance(last.value, ast.Constant)
                and last.value.value is True):
            raise ValueError('缺少正常阶段完成返回值')
    trailing = row.get('trailing_added', 0)
    if trailing:
        body = body[:-trailing]
    module = ast.Module(body=body, type_ignores=[])
    RestoreNames(row['reverse_names'], row.get('outer_continue', False)).visit(module)
    if row.get('remove_self_capture'):
        # import 后立即保存模块/类型引用属于设备生命周期搬移。
        module.body = [n for n in module.body if not (
            isinstance(n, ast.Assign) and len(n.targets) == 1
            and shape(n.targets[0]).get('type') in ('Name', 'Tuple')
            and ast.dump(n.targets[0], include_attributes=False).replace('Store()', 'Load()')
            == ast.dump(n.value, include_attributes=False))]
    return module.body


def audit(root, trees):
    """返回逐项结果；能证明迁移结构，不能证明真实设备运行行为。"""
    report = []

    def add(name, passed, detail):
        """追加源码证据，供主静态检查器统一汇总。"""
        report.append((name, passed, detail))

    metadata = json.loads((root / 'validation/refactor_blocks.json').read_text(encoding='utf-8'))
    baseline_path = root / metadata['source_file']
    baseline = baseline_path.read_text(encoding='utf-8')
    add('refactor:baseline', hashlib.sha256(baseline.encode('utf-8')).hexdigest()
        == metadata['source_sha256'], '保留中文注释版入口源码及其 SHA256，作为本轮迁移依据')
    lines = baseline.splitlines(keepends=True)
    for row in metadata['blocks']:
        original = ast.parse(textwrap.dedent(''.join(
            lines[row['source_start'] - 1:row['source_end']])), feature_version=8).body
        fn = find_method(trees[row['module']], row['method'])
        current = fn.body[1 + row['aliases']:]
        current = normalize(current, row)
        passed = digest(original) == digest(current)
        add('refactor:' + row['module'] + ':' + row['method'], passed,
            '变量改为上下文属性、明确出口改为方法返回后，业务 AST 与迁移前一致')
        if not passed:
            # 输出差异位置，有助于维护者定位；不执行任何业务分支。
            original_dump, current_dump = shape(original), shape(current)
            add('refactor_diff:' + row['method'], False,
                '原语句数={}，当前语句数={}'.format(len(original_dump), len(current_dump)))

    original_tree = ast.parse(baseline, feature_version=8)
    original_loop = next(n for n in ast.walk(original_tree)
                         if isinstance(n, ast.While) and n.lineno == 645)
    current_run = find_method(trees['app/workflow.py'], 'run')
    current_loop = next(n for n in current_run.body if isinstance(n, ast.While))
    old_try = next(n for n in original_loop.body if isinstance(n, ast.Try))
    new_try = next(n for n in current_loop.body if isinstance(n, ast.Try))
    mapping = metadata['blocks'][next(i for i, row in enumerate(metadata['blocks'])
                                     if row['method'] == '_begin')]['reverse_names']
    row = {'reverse_names': mapping}
    old_prefix = original_loop.body[:original_loop.body.index(old_try)]
    new_prefix = current_loop.body[:current_loop.body.index(new_try)]
    add('refactor:stage_budget', digest(old_prefix) == digest(normalize(new_prefix, row)),
        '整场截止、阶段截止、SIGALRM 设置和阶段事件顺序保持')
    tail = next(n.body for n in new_try.body if isinstance(n, ast.If)
                and isinstance(n.test, ast.Name) and n.test.id == 'completed')
    add('refactor:stage_tail', digest(old_try.body[-2:]) == digest(normalize(tail, row)),
        '正常阶段完成后的停止/超时检查保持；原 continue 路径由 False 返回保留')
    return report
