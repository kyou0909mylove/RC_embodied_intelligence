# 中文阅读与维护导航

## 先读入口

main_2026.py 只有 51 行：解析参数、校验配置、选择只读或实机模式、运行状态流程。不加 --real 会在配置分支结束。

实机通过 with RobotSession 管理资源。准备阶段失败也会清理已建立的设备；运行中的超时、Ctrl+C、普通异常和正常结束走统一收尾。

## 要改什么，去哪里

| 需求 | 文件/位置 |
|---|---|
| 物品、标签、坐标或标定值 | config/competition.local.json |
| 配置限制与错误提示 | ConfigurationLoader 对应 _check_* 方法 |
| 任务排序与离场判断 | Workflow.handle_select |
| 完整任务的导航行为 | Workflow.handle_navigate |
| 靠近位姿组织 | Workflow.handle_approach |
| 抓取接口如何调用 | Workflow.handle_grasp |
| 手指持物反馈 | Workflow.handle_grip |
| 收臂后的检查条件 | Workflow.handle_stow |
| 放置动作与打开反馈 | Workflow.handle_place |
| 图像、深度、TF、跨帧关联 | VisionStage._collect_observation |
| 靠近后的目标复核 | VisionStage._recheck |
| 槽位选择与交付确认 | VisionStage._before_delivery / _after_delivery |
| 设备初始化、开始信号与清理 | app/session.py |
| 检查点、日志和最终记录 | app/evidence.py |
| 语法、接口与迁移结构检查 | tools/static_check.py |
| 导航连接、规划和单次移动 | tools/test_navigation.py，见 START_NAV_TEST.md |

现有能力范围内增加物品，需要同步 task_catalog 与区域 targets，匹配真实模型标签并完成标定。增加柜门等新能力仍需真实接口支持。

## 三种数据对象

Configuration 保存配置，RobotSession 保存设备，RunContext 保存当前进度。

| 字段 | 含义 |
|---|---|
| ctx.state / ctx.stage | 下一轮步骤 / 本轮固定步骤 |
| ctx.holding | 当前夹爪判断 |
| ctx.area / ctx.task | 当前区域与物品任务 |
| ctx.target_map | 视觉地图位置，用于底盘靠近 |
| ctx.destination | 本次导航位姿 |
| ctx.nav_kind / ctx.nav_after | 导航用途 / 到达后的阶段 |
| ctx.scan_mode / ctx.grip_after | 扫描目的 / 手指检查后的阶段 |
| ctx.slot / ctx.attempt | 放置位 / 本次尝试记录 |
| ctx.inventory / ctx.deliveries | 数量台账 / 已复核交付 |
| ctx.deadline / ctx.operation_deadline | 整场 / 当前阶段截止 |

State、HoldingState、ScanMode 在 context.py 定义，继承字符串类型，JSON 中仍保存原值。阶段结束会核对并规范状态名称。

holding 的 EMPTY 表示流程认为空闲；UNKNOWN 表示可能已抓到但未确认；HOLD_INDICATED 是位置反馈辅助证据；RELEASE_UNVERIFIED 是已松爪但未确认交付。

grip_after="SCORE_VIEW" 是检查后安排前往得分区观察点的转向标记，不是独立状态处理器。

## 一趟任务的阅读顺序

1. SELECT 选区域，NAVIGATE 到搜索点。
2. SCAN/SEARCH 选可见目标，APPROACH 算靠近点，再导航。
3. SCAN/RECHECK 复核目标，GRASP 调用原抓取。
4. GRIP 检查、STOW 收臂、再 GRIP，随后运回观察点。
5. SCAN/SLOT_BEFORE 建基线并选槽位；导航到投放位。
6. 再 GRIP，PLACE 投放并收臂，返回观察点。
7. SCAN/SLOT_AFTER 确认新增物品，才增加交付数量。
8. 回到 SELECT，继续任务或按原条件离场/停止。

## 阶段方法的返回值

阶段直接更新 ctx.state 等进度，保持原更新时机，例如抓取动作之前就设 holding=UNKNOWN。

- True：正常走完阶段，主循环进行阶段结束检查。
- False：对应原 while 中的 continue，已决定切换或重试；仍保存阶段检查点。
- 抛异常：进入统一停止与清理。

这里的 True 不表示抓取成功。原 catch_* 的 None 返回约定保持，持物与交付由后续检查判断。

## 计时与停止

600 秒常量集中在 context.py。正式信号只设置一次整场截止；Workflow.run 设置更早的阶段截止。各方法与原 action 继承同一个 gate，没有重新获得阶段预算。

检查点写入也包含在阶段时间内。清理取消请求、发送零速度、关闭资源和保存记录，不新增回 home 或松爪动作。

## 配置校验顺序

ConfigurationLoader.load 依次调用 _read、_check_catalog、_check_structure、_collect_areas、_check_shapes、_check_parameters、_check_relations、_check_files。

清单模式提前返回；配置错误交给入口返回 2。各校验方法都只处理数据与文件。

## 修改后如何检查

```bash
python3 -S tools/static_check.py
python3 -S main_2026.py --check-config
```

主动修改算法后，迁移 AST 比较可能失败。先审查、说明差异，再维护对应基线；原接口与受保护常量继续独立检查。

validation/refactor_baseline_main.py.txt 是拆分前入口，仅用于审查，不是运行入口。当前结果看 VALIDATION.md，历史报告在 docs/history_before_refactor/。
