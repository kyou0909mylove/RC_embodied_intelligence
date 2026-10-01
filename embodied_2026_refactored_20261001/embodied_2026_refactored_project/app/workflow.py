# -*- coding: utf-8 -*-
"""比赛状态控制器。每个阶段一个方法；计时和检查点由主循环统一管理。"""
import math
import signal
import time
from .vision import VisionStage


class Workflow:
    """调度现有技能；动作成功、持物反馈、视觉交付分别判断。"""

    def __init__(self, context, session):
        """接收已经准备完成的会话，建立阶段处理表。"""
        self.context = context
        self.session = session
        self.vision_stage = VisionStage(context, session)
        self.handlers = {
            "SELECT": self.handle_select,
            "NAVIGATE": self.handle_navigate,
            "SCAN": self.handle_scan,
            "APPROACH": self.handle_approach,
            "GRASP": self.handle_grasp,
            "GRIP": self.handle_grip,
            "STOW": self.handle_stow,
            "PLACE": self.handle_place,
        }

    def _begin(self):
        """按原公式计算预算，首次目的地为 entry。"""
        ctx = self.context
        cfg = self.session.config.values
        est = self.session.config.values["estimates"]
        lim = self.session.config.values["limits"]
        est, lim, vision = cfg["estimates"], cfg["limits"], cfg["vision"]
        # 【预算解释】返航含去观察位、去投放位、再回观察位，共3次导航；投放前后共2次扫描，还有投放和持物检查。
        ctx.return_cost = 3 * est["nav_s"] + 2 * est["scan_s"] + est["place_s"] + est["grip_s"]
        ctx.exit_cost = est["exit_s"] + est["margin_s"]
        ctx.limits = {"SELECT": lim["local_s"], "APPROACH": lim["local_s"],
                  "NAVIGATE": lim["nav_s"], "SCAN": lim["scan_s"], "GRASP": lim["grasp_s"],
                  "GRIP": lim["grip_s"], "STOW": lim["arm_s"], "PLACE": lim["arm_s"]}
        ctx.destination = cfg["locations"]["entry"]
        ctx.nav_kind, ctx.nav_after, ctx.state = "entry", "SELECT", "NAVIGATE"
        ctx.scan_mode = ctx.grip_after = None
        ctx.attempt = None
        ctx.before_zone = None

    def run(self):
        """固定整场截止，每轮执行一个阶段；检查点包含原 continue 路径。"""
        ctx = self.context
        gate = self.session.gate
        rospy = self.session.rospy
        self._begin()
        ctx.normalize_states()
        while ctx.state != "STOP" and not rospy.is_shutdown():
            gate.deadline = ctx.deadline
            gate.check()
            ctx.now = time.monotonic()
            ctx.remaining = ctx.deadline - ctx.now
            if ctx.remaining <= 0:
                raise TimeoutError("比赛 600 秒到时")
            # stage固定本轮步骤名称，即使后面修改state，本轮截止/记录仍对应原来的步骤。
            ctx.stage = ctx.state
            # 取“整场截止”和“本阶段预算截止”中更早的那个，防止接近600秒时还再拿完整阶段时间。
            ctx.operation_deadline = min(ctx.deadline, ctx.now + ctx.limits[ctx.stage])
            gate.deadline = ctx.operation_deadline
            gate.check()
            signal.setitimer(signal.ITIMER_REAL, max(0.001, ctx.operation_deadline - ctx.now))
            ctx.record["events"].append({"event": "stage", "state": ctx.stage,
                                      "elapsed_s": round(ctx.now - ctx.started, 3), "holding": ctx.holding})
            rospy.loginfo("阶段=%s 剩余=%.1fs 持物=%s 已复核交付=%d", ctx.stage, ctx.remaining, ctx.holding, len(ctx.deliveries))
            try:
                completed = self.handlers[ctx.stage]()
                ctx.normalize_states()
                if completed:
                    gate.check()
                    if time.monotonic() >= ctx.operation_deadline:
                        raise TimeoutError("阶段结束时预算耗尽：" + ctx.stage)
            finally:
                try:
                    self.session.evidence.checkpoint(ctx, gate, self.session.config.slots)
                finally:
                    signal.setitimer(signal.ITIMER_REAL, 0)
        if rospy.is_shutdown() and ctx.state != "STOP":
            raise RuntimeError("ROS 中断")

    def handle_select(self):
        """按配额、次数、预算和收益选择区域，或进入离场/停止。"""
        ctx = self.context
        areas = self.session.config.areas
        cfg = self.session.config.values
        est = self.session.config.values["estimates"]
        slots = self.session.config.slots
        strategy = self.session.config.strategy
        if ctx.holding != "EMPTY":
            raise RuntimeError("持物状态不明确，禁止选择下一次抓取")
        ranked = []
        for i, a in enumerate(areas):
            if a["visits"] >= a["visit_limit"]:
                continue
            costs = []
            for t in a["targets"]:
                item = ctx.inventory[t["catalog_id"]]
                if item["delivered"] >= item["expected"]:
                    item["status"] = "COMPLETE"
                    continue
                if (item["pregrasp_failures"] >= strategy["max_pregrasp_failures"]
                        or item["grasp_attempts"] >= strategy["max_grasp_attempts"]):
                    item["status"] = "ATTEMPT_LIMIT"
                    continue
                cycle = (2 * est["nav_s"] + 2 * est["scan_s"]
                         + t["pickup_estimate_s"] + ctx.return_cost)
                if a["cycle_mean_s"] is not None:
                    cycle = max(cycle, 1.2 * a["cycle_mean_s"])
                if cycle + ctx.exit_cost <= ctx.remaining:
                    # 平滑估计：初始先验占两个虚拟样本，实际确认交付提高值，失败和尝试降低值；这只是调度启发式。
                    probability = (2 * t["success_prior"] + item["delivered"]) / (
                        2 + item["grasp_attempts"] + item["pregrasp_failures"])
                    bias = strategy["level1_weight"] if t["level"] == 1 else 1.0
                    if not ctx.deliveries and t["level"] == 1:
                        bias *= strategy["bootstrap_level1_weight"]
                    score = (30 if t["level"] == 1 else 40) * probability * bias
                    costs.append(score / cycle)
            if costs:
                ranked.append((max(costs) / (1 + a["visits"]), -a["visits"], -i, i))
        free_slot = any(s["status"] == "AVAILABLE" for s in slots)
        # 只有至少一件已确认交付才考虑自主离场；退出触发阈值与真正需要的退出预算是两件事。
        should_exit = ctx.deliveries and (ctx.remaining <= est["exit_trigger_s"] or not ranked or not free_slot)
        if should_exit:
            # 触发阈值与最低可用预算分开；剩余 44.9 秒不会因低于 55 秒而被拒绝。
            if ctx.remaining >= ctx.exit_cost:
                ctx.destination = cfg["locations"]["exit"]
                ctx.nav_kind, ctx.nav_after, ctx.state = "exit", "STOP", "NAVIGATE"
            else:
                ctx.record["events"].append({"event": "insufficient_exit_budget"})
                ctx.state = "STOP"
            return False
        if not ranked or not free_slot:
            ctx.record["events"].append({"event": "no_feasible_cycle", "delivery_gate": bool(ctx.deliveries)})
            ctx.state = "STOP"
            return False
        # ranked每个元组保存得分和区域索引；max取最高分，末尾索引用于拿到那个区域。
        ctx.area = areas[max(ranked)[3]]
        ctx.record["events"].append({"event": "selected_area", "area": ctx.area["id"],
                                 "expected_value_per_s": max(ranked)[0]})
        ctx.area["visits"] += 1
        ctx.cycle_started = time.monotonic()
        ctx.task = ctx.attempt = None
        ctx.destination = ctx.area["search_location"]
        ctx.scan_mode = "SEARCH"
        ctx.nav_kind, ctx.nav_after, ctx.state = "search", "SCAN", "NAVIGATE"
        return True

    def handle_navigate(self):
        """检查位置和规划，有限等待；空爪搜索失败可换任务。"""
        ctx = self.context
        GoalStatus = self.session.GoalStatus
        base = self.session.base
        gate = self.session.gate
        navigator = self.session.navigator
        planner = self.session.planner
        rospy = self.session.rospy
        goal = navigator.set_goal("map", ctx.destination[0], ctx.destination[1])
        goal.target_pose.header.stamp = rospy.Time.now()
        pose = planner.get_robot_pose()
        gate.check()
        if time.monotonic() >= ctx.operation_deadline:
            raise TimeoutError("定位调用超时")
        # 先有真实定位，再查规划路径；返回再查截止，迟到的True不能授权发目标。
        reachable = pose is not None and planner.validate_goal(pose, goal.target_pose)
        # 服务即使迟到返回 True，也不能继续派发导航，更不能 wait(Duration(0))。
        gate.check()
        if time.monotonic() >= ctx.operation_deadline:
            raise TimeoutError("路径检查超时，未发送导航目标")
        arrived = False
        if reachable:
            wait_s = ctx.operation_deadline - time.monotonic() - 0.02
            if wait_s <= 0:
                raise TimeoutError("导航预算耗尽")
            # 真正向move_base发送导航目标；client已被GuardedClient包装。
            navigator.client.send_goal(goal)
            wait_s = ctx.operation_deadline - time.monotonic() - 0.01
            if wait_s <= 0:
                raise TimeoutError("导航发送后预算耗尽")
            try:
                arrived = bool(navigator.client.wait_for_result(rospy.Duration(wait_s)))
            except TimeoutError:
                # 门控会把 action 失败变为异常。仅及时返回的终态失败可换点；
                # 外部停止、到时、等待未完成均继续抛出并停止整场流程。
                gate.check()
                nav_status = navigator.client.get_state()
                if nav_status in (2, 4, 5, 8, 9):
                    arrived = False
                else:
                    raise
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("导航结果迟到")
            arrived = arrived and navigator.client.get_state() == GoalStatus.SUCCEEDED
        if not arrived:
            navigator.stop()
            base.stop()
            ctx.record["events"].append({"event": "navigation_failed", "kind": ctx.nav_kind})
            if ctx.attempt is not None:
                ctx.attempt["status"] = "NAVIGATION_FAILED"
            # 仅空闲夹爪下的搜索/靠近失败允许换任务；携物返回或投放导航失败时保守停止。
            if ctx.holding == "EMPTY" and ctx.nav_kind in ("search", "approach"):
                ctx.state = "SELECT"
            else:
                raise RuntimeError("入场/携物回程/离场导航失败")
        else:
            base.stop()
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("导航到达确认迟到")
            if ctx.nav_kind == "exit":
                ctx.record["exit_completed"] = True
            ctx.state = ctx.nav_after
        return True

    def handle_scan(self):
        """委托视觉模块执行原四种扫描目的。"""
        return self.vision_stage.run()

    def handle_approach(self):
        """计算朝向目标的靠近位姿；实际移动交给 NAVIGATE。"""
        ctx = self.context
        calculate_facing_goal = self.session.calculate_facing_goal
        cfg = self.session.config.values
        gate = self.session.gate
        planner = self.session.planner
        pose = planner.get_robot_pose()
        goal_pose = calculate_facing_goal(pose, ctx.target_map, ctx.task["standoff_m"]) if pose is not None else None
        gate.check()
        if time.monotonic() >= ctx.operation_deadline:
            raise TimeoutError("靠近位姿计算迟到")
        x0, x1, y0, y1 = cfg["field_bounds"]
        if (goal_pose is None or not all(math.isfinite(float(x)) for p in goal_pose for x in p)
                or not x0 <= goal_pose[0][0] <= x1 or not y0 <= goal_pose[0][1] <= y1):
            ctx.attempt["status"] = "APPROACH_UNAVAILABLE"
            ctx.state = "SELECT"
            return False
        ctx.destination = goal_pose
        ctx.scan_mode = "RECHECK"
        ctx.nav_kind, ctx.nav_after, ctx.state = "approach", "SCAN", "NAVIGATE"
        return True

    def handle_grasp(self):
        """按 surface 调用原抓取方法；None 返回不作为成功标志。"""
        ctx = self.context
        GoalStatus = self.session.GoalStatus
        base = self.session.base
        gate = self.session.gate
        ground_detector = self.session.ground_detector
        kinova = self.session.kinova
        vision = self.session.config.values["vision"]
        if ctx.holding != "EMPTY":
            raise RuntimeError("非空闲夹爪禁止新抓取")
        if ctx.deadline - time.monotonic() < ctx.task["pickup_estimate_s"] + ctx.return_cost + ctx.exit_cost:
            ctx.attempt["status"] = "DEFERRED_TIME"
            ctx.state = "SELECT"
            return False
        base.stop()
        if ctx.task["surface"] == "ground":
            kinova.goto_detect()
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("地面观察动作迟到")
            if kinova.client_arm.get_state() != GoalStatus.SUCCEEDED:
                raise RuntimeError("未到达地面观察位")
            ground_result = ground_detector.detect_targets(
                target_items=[ctx.task["grasp_label"]], max_retry=3, show_window=False)
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("地面检测迟到")
            if (ground_result is None or ground_result.name != ctx.task["grasp_label"]
                    or not all(math.isfinite(float(x)) for x in
                               (ground_result.x, ground_result.y, ground_result.z, ground_result.angle))
                    or not 0.1 <= ground_result.z <= vision["depth_max_m"]):
                ctx.attempt["status"] = "GROUND_DETECTION_FAILED"
                ctx.inventory[ctx.task["catalog_id"]]["pregrasp_failures"] += 1
                # 原观察位仍须回 home 才能移动；此时尚未调用抓取。
                ctx.grip_after, ctx.state = "SELECT", "STOW"
                return False
            # commands保存“方法对象、参数字典”；下面command(**kwargs)才真正调用它。
            commands = [(kinova.catch_ground, {"result": ground_result})]
        elif ctx.task["surface"] == "table":
            commands = [(kinova.catch_table, {"target": [ctx.task["grasp_label"]]})]
        else:
            commands = [(kinova.catch_table_short, {"target": [ctx.task["grasp_label"]]})]
        # 开始抓取后先标为UNKNOWN：发生异常时不能假设空爪，然后继续抓第二件。
        ctx.holding = "UNKNOWN"
        ctx.attempt["status"] = "GRASP_STARTED"
        ctx.inventory[ctx.task["catalog_id"]]["grasp_attempts"] += 1
        # 原 catch_* 返回 None，绝不据此判定成功或失败。
        # 原抓取动作结束后追加闭爪请求；是否支持持物由后续GRIP反馈样本判断。
        commands.append((kinova.close_finger, {}))
        table_detection_failed = False
        for command, kwargs in commands:
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("抓取预算耗尽")
            command(**kwargs)
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("原抓取/夹爪调用迟到")
            if (kinova.client_arm.get_state() != GoalStatus.SUCCEEDED
                    or kinova.client_finger.get_state() != GoalStatus.SUCCEEDED):
                raise RuntimeError("原动作最后的 action 未成功；停止后续动作")
            if (ctx.task["surface"] != "ground" and command != kinova.close_finger
                    and getattr(kinova, "last_grasp_detection", "NOT_REPORTED") is None):
                # 原方法的 if result 分支未执行，此前的空爪状态仍成立。
                # 先收拢再换任务；不把 None 返回值本身当作失败，也不闭空爪。
                ctx.holding = "EMPTY"
                ctx.attempt["status"] = "TABLE_DETECTION_FAILED"
                ctx.inventory[ctx.task["catalog_id"]]["pregrasp_failures"] += 1
                ctx.inventory[ctx.task["catalog_id"]]["grasp_attempts"] -= 1
                ctx.grip_after, ctx.state = "SELECT", "STOW"
                table_detection_failed = True
                break
        if table_detection_failed:
            return False
        ctx.grip_after, ctx.state = "STOW", "GRIP"
        return True

    def handle_grip(self):
        """采集新反馈；同样至少两根手指在范围内且稳定才通过。"""
        ctx = self.context
        FingerPosition = self.session.FingerPosition
        cfg = self.session.config.values
        gate = self.session.gate
        kinova = self.session.kinova
        rospy = self.session.rospy
        samples = []
        for _ in range(cfg["gripper"]["samples"]):
            wait_s = min(1.0, ctx.operation_deadline - time.monotonic() - 0.01)
            if wait_s <= 0:
                raise TimeoutError("夹爪反馈预算耗尽")
            feedback = rospy.wait_for_message("/j2n6s300_driver/out/finger_position", FingerPosition, timeout=wait_s)
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("夹爪反馈迟到")
            kinova.setCurrentFingerPosition(feedback)
            samples.append([float(x) for x in kinova.currentFingerPosition])
        valid = all(len(s) == 3 and all(math.isfinite(x) and 0 <= x <= 6800 for x in s) for s in samples)
        # 按手指索引i比较所有样本，防止每个样本合格的手指不同却误判为持续夹持。
        same_fingers = (valid and sum(
            all(ctx.task["hold_min"] <= s[i] <= ctx.task["hold_max"] for s in samples) and
            max(s[i] for s in samples) - min(s[i] for s in samples) <= cfg["gripper"]["stability_turn"]
            for i in range(3)) >= 2)
        ctx.attempt.setdefault("grip_samples", []).append(samples)
        if not same_fingers:
            ctx.holding = "UNKNOWN"
            ctx.attempt["status"] = "HOLD_UNKNOWN"
            raise RuntimeError("夹爪反馈不足以支持持物判断；不重抓、不松爪、不离场")
        ctx.holding = "HOLD_INDICATED"  # 辅助判据，不等同于力传感器确认
        ctx.attempt["status"] = "HOLD_INDICATED"
        if ctx.grip_after == "SCORE_VIEW":
            ctx.destination = cfg["locations"]["score_view"]
            ctx.scan_mode = "SLOT_BEFORE"
            ctx.nav_kind, ctx.nav_after, ctx.state = "score_view_before", "SCAN", "NAVIGATE"
        else:
            ctx.state = ctx.grip_after
        return True

    def handle_stow(self):
        """调用原 goto_home 收臂，确认完成后才允许携物导航。"""
        ctx = self.context
        GoalStatus = self.session.GoalStatus
        gate = self.session.gate
        kinova = self.session.kinova
        gate.check()
        if time.monotonic() >= ctx.operation_deadline:
            raise TimeoutError("回 home 预算耗尽")
        kinova.goto_home()
        gate.check()
        if time.monotonic() >= ctx.operation_deadline:
            raise TimeoutError("回 home 返回迟到")
        if kinova.client_arm.get_state() != GoalStatus.SUCCEEDED:
            raise RuntimeError("回 home 失败，不开始底盘导航")
        if ctx.holding == "EMPTY":  # 地面检测失败且尚未抓取
            ctx.state = "SELECT"
        else:
            ctx.grip_after, ctx.state = "SCORE_VIEW", "GRIP"
        return True

    def handle_place(self):
        """先到标定姿态，后松爪与检查；仍等待投放后视觉确认。"""
        ctx = self.context
        FingerPosition = self.session.FingerPosition
        GoalStatus = self.session.GoalStatus
        cfg = self.session.config.values
        gate = self.session.gate
        kinova = self.session.kinova
        rospy = self.session.rospy
        if ctx.holding != "HOLD_INDICATED" or ctx.slot["status"] != "RESERVED":
            raise RuntimeError("放置前条件不满足")
        # 从已校准 drop 停车位执行既有 arm_run；不调用带固定交接动作的 open_finger。
        for pose in ctx.slot["drop_poses"]:
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("放置姿态预算耗尽")
            kinova.arm_run(unit="mdeg", pose_target=pose, relative=False)
            gate.check()
            if time.monotonic() >= ctx.operation_deadline:
                raise TimeoutError("放置动作迟到；禁止继续松爪")
            if kinova.client_arm.get_state() != GoalStatus.SUCCEEDED:
                raise RuntimeError("放置姿态未到达；禁止松爪")
        gate.check()
        if time.monotonic() >= ctx.operation_deadline:
            raise TimeoutError("松爪前预算耗尽")
        # 此时只是开始松爪，物品是否真实落入区内还不知道；不能直接把库存+1。
        ctx.holding = "RELEASE_UNVERIFIED"
        kinova.finger_run(unit="percent", finger_target=[5, 5, 5], relative=False)
        gate.check()
        if time.monotonic() >= ctx.operation_deadline:
            raise TimeoutError("松爪返回迟到；不登记交付")
        if kinova.client_finger.get_state() != GoalStatus.SUCCEEDED:
            raise RuntimeError("松爪 action 失败")
        # 收集配置数量的打开反馈，三根手指都要在范围内，且跨样本变化不能超过稳定阈值。
        open_samples = []
        for _ in range(cfg["gripper"]["samples"]):
            gate.check()
            wait_s = min(1.0, ctx.operation_deadline - time.monotonic() - 0.01)
            if wait_s <= 0:
                raise TimeoutError("松爪反馈预算耗尽")
            feedback = rospy.wait_for_message("/j2n6s300_driver/out/finger_position", FingerPosition, timeout=wait_s)
            gate.check()
            kinova.setCurrentFingerPosition(feedback)
            sample = [float(x) for x in kinova.currentFingerPosition]
            open_samples.append(sample)
            ctx.attempt["open_samples"] = open_samples
            if (len(sample) != 3 or not all(math.isfinite(x) and
                    0 <= x <= cfg["gripper"]["open_max"] for x in sample)):
                raise RuntimeError("夹爪未达到标定的打开范围")
        if any(max(s[i] for s in open_samples) - min(s[i] for s in open_samples)
               > cfg["gripper"]["stability_turn"] for i in range(3)):
            raise RuntimeError("松爪反馈不稳定，不登记交付")
        gate.check()
        if time.monotonic() >= ctx.operation_deadline:
            raise TimeoutError("放置后回 home 预算耗尽")
        kinova.goto_home()
        gate.check()
        if time.monotonic() >= ctx.operation_deadline:
            raise TimeoutError("放置后回 home 迟到")
        if kinova.client_arm.get_state() != GoalStatus.SUCCEEDED:
            raise RuntimeError("放置后回 home 失败")
        ctx.destination = cfg["locations"]["score_view"]
        ctx.scan_mode = "SLOT_AFTER"
        ctx.nav_kind, ctx.nav_after, ctx.state = "score_view_after", "SCAN", "NAVIGATE"
        return True
