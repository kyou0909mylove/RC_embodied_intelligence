# 本轮模块拆分说明

## 来源与范围

以中文注释版 embodied_2026_commented_project 为基准。用户本轮授权拆分并打包，因此新增配置、上下文、会话、流程、视觉和记录辅助类/方法。原机器人技能继续复用，10 个旧模块与 safety.py 对照基准字节一致。

## 迁移位置

| 原入口职责 | 新位置 |
|---|---|
| 配置校验 | app/configuration.py，8 个步骤 |
| 跨阶段变量与状态 | app/context.py |
| 场次、准备、开始与清理 | app/session.py |
| 状态循环及动作阶段 | app/workflow.py |
| 多帧观察与四种视觉判据 | app/vision.py |
| 检查点、JSONL、最终记录 | app/evidence.py |

入口名称与选项、schema 3、配置键、抓取 None 返回、600 秒总截止、阶段预算、持物状态和交付判断保持。

阶段返回 True/False 区分普通结束与原 continue；持物判断仍在原动作前更新。状态核对转换为兼容字符串的枚举值。

会话采用上下文管理器，准备失败也清理部分初始化对象；信号逐个保存与恢复。清理不派发新运动。

## 检查与追溯

接口检查从单一入口扩大到入口和全部 app 模块，新增相对导入、导入边界、名称绑定、上下文字段及迁移 AST 核对。

原 95 个接口声明与受保护数值依旧以 legacy_contracts.json 为依据。本轮迁移依据为 refactor_baseline_main.py.txt 与 refactor_blocks.json。

REFACTOR.patch 是本轮代码变化；LEGACY_FIXES.patch 和 ANNOTATIONS.patch 是此前来源记录。之前说明与报告在 docs/history_before_refactor/。

导航工具与网络脚本沿用已交付文件，操作说明适配完整包目录。本轮只做静态检查、接口核对、只读 CLI/导入隔离与打包，没有运行机器人或模拟回归。
