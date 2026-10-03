# 本文件保留已上传的原始接口，当前主入口/导航测试不直接调用这些硬件方法。
# 真实接入前需复核依赖、坐标、模型标签和标定；None/action 完成不代表持物成功。
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ========== 中文阅读说明 ==========
# SmartGoalFinder 从 TF 读取机器人当前位置，并向 make_plan 请求路径检查。
# 保留原 TF 取姿态/检查规划接口供后续开发；当前固定点导航流程不自动调用它。
# find_best_goal 保留了往年“在人附近寻找候选位置”的代码，不是当前主流程入口。
# 能请求到规划路径只表示规划检查通过，不能证明真实导航或避障一定成功。
# ==================================

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.srv import GetPlan, GetPlanRequest
import math
import numpy as np
import time

"""
该方法在人周围一个安全的环形区域内 找到一个最佳的可达导航目标点
传入人的三维坐标 返回一个可导航点

created by zx 2025-10-03
"""

# 【类 SmartGoalFinder】
class SmartGoalFinder:
    """
    一个智能寻找并验证导航目标的ROS节点。
    它会在人的周围一个安全的环形区域内，找到一个最佳的可达导航目标点。
    """
    # 【函数/方法 SmartGoalFinder.__init__】
    # 建立 map/base_link 的 TF 监听，并连接 /move_base/make_plan。
    def __init__(self):
        # rospy.init_node('smart_goal_finder', anonymous=True)

        self.robot_base_frame = 'base_link'
        self.map_frame = 'map'

        self.MAX_SEARCH_RADIUS = 0.6
        self.MIN_SEARCH_RADIUS = 0.3
        
        self.RADIUS_STEP = 0.05
        self.ANGULAR_STEP = 10

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.make_plan_service_name = "/move_base/make_plan"
        rospy.loginfo(f"等待服务 '{self.make_plan_service_name}'...")
        try:
            rospy.wait_for_service(self.make_plan_service_name, timeout=5.0)
            self.make_plan_client = rospy.ServiceProxy(self.make_plan_service_name, GetPlan)
            rospy.loginfo("服务连接成功!")
        except rospy.ROSException as e:
            rospy.logerr(f"连接服务失败: {e}")
            rospy.signal_shutdown("无法连接到 make_plan 服务")
            return

    # 【函数/方法 SmartGoalFinder.get_robot_pose】
    # 读取 map 到 base_link 的最新 TF，返回 PoseStamped。
    # 单次读取最多等待 5 秒；若主流程预算更短，以主流程截止为准。
    # 短暂缓存未就绪只记录等待，最终失败返回 None，不能用零位姿继续导航。
    def get_robot_pose(self):
        """在有限等待内取得 map 位姿；接口签名和 PoseStamped/None 返回保持。"""
        gate = getattr(self, 'deadline_gate', None)
        deadline = time.monotonic() + 5.0
        if gate is not None:
            gate.check()
            deadline = min(deadline, gate.deadline)
        last_error = None
        waiting = False
        while not rospy.is_shutdown():
            # 用系统单调时钟计时；ROS 模拟时间暂停或设备墙钟差异不会延长等待。
            # 每轮检查同一个停止/截止门控，停止异常直接交给主流程收尾。
            if gate is not None:
                gate.check()
            if time.monotonic() >= deadline:
                break
            try:
                # Time(0) 取最新可用变换；Duration(0) 使本次查询不阻塞。
                # 等待由外面的有界轮询完成，避免 TF 内部按 ROS 时间等待。
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.robot_base_frame,
                    rospy.Time(0),
                    rospy.Duration(0.0)
                )
                p = transform.transform.translation
                q = transform.transform.rotation
                if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                    raise ValueError('TF 位姿包含非有限数值')
                if abs(math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) - 1.0) > 0.01:
                    raise ValueError('TF 位姿四元数未归一化')
                # 迟到的查询结果不能授权后续导航；成功返回前再检查一次。
                if gate is not None:
                    gate.check()
                if time.monotonic() >= deadline:
                    last_error = 'TF 查询返回时等待期限已到'
                    break
                pose = PoseStamped()
                pose.header.frame_id = self.map_frame
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = p.x
                pose.pose.position.y = p.y
                pose.pose.position.z = p.z
                pose.pose.orientation = q
                if waiting:
                    rospy.loginfo('TF 缓存已就绪，取得机器人 map 位姿')
                return pose
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException, ValueError) as e:
                last_error = e
                if not waiting:
                    rospy.loginfo('正在等待 map -> base_link 位姿，单次最多 5 秒')
                    waiting = True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(0.05, remaining))

        if not rospy.is_shutdown():
            rospy.logerr('等待机器人位姿超时，未使用默认坐标：%s', last_error)
        return None

    # 【函数/方法 SmartGoalFinder.validate_goal】
    # 输入起点与终点 PoseStamped，调用 make_plan；规划路径非空返回 True。
    # False 表示本次预检查没得到路径，不代表永久不可达。
    def validate_goal(self, start_pose, goal_pose):
        """
        调用 move_base/make_plan 服务来验证一个目标点是否可达。
        如果服务返回的路径不为空, 则认为该点可达。
        """
        req = GetPlanRequest()
        req.start = start_pose
        req.goal = goal_pose
        req.tolerance = 0.1  # 容忍度设小一点，确保规划到目标点附近
        try:
            res = self.make_plan_client(req)
            if res.plan.poses:  # 如果路径点列表不为空
                return True
            else:
                return False
        except rospy.ServiceException as e:
            rospy.logwarn(f"调用 make_plan 服务异常: {e}")
            return False

    # 【函数/方法 SmartGoalFinder.find_best_goal】
    # 旧候选搜索：在目标附近改变距离和角度，找到首个规划可达位姿。
    # 实际输入使用列表位置数据，实际返回 [[x,y,z],[qx,qy,qz,qw]] 或 None。
    def find_best_goal(self, person_pose_stamped):
        """
        主逻辑函数：围绕人的位置搜索最佳导航目标点。
        
        Args:
            person_pose_stamped(list): 人在map坐标系下的位姿。
        
        Returns:
            PoseStamped: 找到的最佳导航目标位姿, 如果找不到则返回 None。
        """
        # 1. 获取机器人当前位姿作为路径规划的起点
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return None

        person_point_x = person_pose_stamped[0]
        person_point_y = person_pose_stamped[1]
        person_point_z = person_pose_stamped[2]

        robot_point = robot_pose.pose.position

        # 2. 计算机器人到人的初始角度，作为搜索的0度方向
        initial_angle = math.atan2(person_point_y - robot_point.y, person_point_x - robot_point.x)

        # 3. 从外环向内环进行迭代搜索
        current_radius = self.MAX_SEARCH_RADIUS
        while current_radius >= self.MIN_SEARCH_RADIUS:
            rospy.loginfo(f"正在半径 {current_radius:.2f}m 处搜索...")

            # 4. 在当前半径的圆上，从0度开始向两侧扩展搜索角度
            # 角度偏移顺序: 0, +15, -15, +30, -30, ...
            for angle_offset_multiplier in range(0, int(math.pi / self.ANGULAR_STEP) + 1):
                for sign in ([1, -1] if angle_offset_multiplier > 0 else [1]):
                    
                    angle_offset = angle_offset_multiplier * self.ANGULAR_STEP * sign
                    current_angle = initial_angle + angle_offset

                    # 计算候选点的坐标
                    goal_x = person_point_x - current_radius * math.cos(current_angle)
                    goal_y = person_point_y - current_radius * math.sin(current_angle)

                    # 计算朝向人的姿态 (yaw角)
                    face_person_yaw = math.atan2(person_point_y - goal_y, person_point_x - goal_x)
                    q = tf_conversions.transformations.quaternion_from_euler(0, 0, face_person_yaw)
                    
                    # 构建候选目标位姿
                    candidate_goal = PoseStamped()
                    candidate_goal.header.frame_id = self.map_frame
                    candidate_goal.header.stamp = rospy.Time.now()
                    candidate_goal.pose.position.x = goal_x
                    candidate_goal.pose.position.y = goal_y
                    candidate_goal.pose.position.z = 0.138 # Z轴高度与人保持一致   0.138
                    candidate_goal.pose.orientation.x = q[0]
                    candidate_goal.pose.orientation.y = q[1]
                    candidate_goal.pose.orientation.z = q[2]
                    candidate_goal.pose.orientation.w = q[3]

                    # 5. 验证该候选点是否可达
                    rospy.loginfo(f"  -> 验证角度: {math.degrees(current_angle):.1f}°, "
                                f"坐标: ({goal_x:.2f}, {goal_y:.2f})")
                    time.sleep(0.1)
                    if self.validate_goal(robot_pose, candidate_goal):
                        rospy.loginfo(f"成功找到有效目标点！半径: {current_radius:.2f}m, "
                                      f"坐标: ({goal_x:.2f}, {goal_y:.2f})")
                        goodgoal = [[goal_x,goal_y,0.138],[q[0],q[1],q[2],q[3]]]
                        return goodgoal

            # 如果当前半径的所有点都无效，则向内缩小半径
            current_radius -= self.RADIUS_STEP
        
        rospy.logwarn("在整个定义的环形区域内都未找到有效的导航目标点。")
        return None
