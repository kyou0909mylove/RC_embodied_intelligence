#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ========== 中文阅读说明 ==========
# 本文件只计算“机器人应该停在哪里、面朝哪边”，本身不发送导航动作。
# 输入是机器人当前位姿、物品 map 坐标和保持距离；输出是位置列表与四元数列表。
# 代码中的 Pose 注解是历史写法；实际访问的是 robot_pose.pose.position。
# 返回值实际为 [[x,y,z],[qx,qy,qz,qw]]，并非直接返回 ROS Pose 消息。
# ==================================

"""
计算机器人面向目标点的导航目标位置和姿态。
传入机器人当前位姿和目标点的地图坐标，返回一个新的位姿，使机器人面向目标点并保持一定距离。
理论上保证相对位置不变

created by zx 2025-10-18
"""

import math
import tf_conversions
from geometry_msgs.msg import Pose, Point, Quaternion

# 【函数/方法 calculate_facing_goal】
# 输入 robot_pose、target_point_map=[x,y,z]、distance（米）。
# 先从物品沿机器人方向后退 distance，得到停车点，再计算朝向物品的 yaw。
# 输出 [[x,y,z],[qx,qy,qz,qw]]；机器人与目标太接近时返回 None。
def calculate_facing_goal(robot_pose: Pose, target_point_map: list, distance: float = 0.3) -> Pose:
    # 1. 提取机器人和目标点的2D坐标
    robot_x = robot_pose.pose.position.x
    robot_y = robot_pose.pose.position.y
    target_x, target_y, _ = target_point_map

    # 2. 计算从机器人指向目标点的向量
    approach_vector_x = target_x - robot_x
    approach_vector_y = target_y - robot_y
    
    vector_magnitude = math.sqrt(approach_vector_x**2 + approach_vector_y**2)

    # 如果机器人与目标点重合，无法计算
    if vector_magnitude < 0.01:
        print("错误：机器人与目标点距离过近。")
        return None

    # 3. 计算单位方向向量
    unit_vector_x = approach_vector_x / vector_magnitude
    unit_vector_y = approach_vector_y / vector_magnitude

    # 4. 计算导航目标点的坐标 (从目标点沿反方向后退)
    # 停车点在物品面向机器人这一侧；distance为物品到停车点的距离。
    goal_x = target_x - distance * unit_vector_x
    goal_y = target_y - distance * unit_vector_y

    # 5. 计算导航目标的姿态 (使其朝向目标点)
    face_target_yaw = math.atan2(target_y - goal_y, target_x - goal_x)
    # 把平面朝向yaw弧度转成四元数，供导航消息使用。
    q_tuple = tf_conversions.transformations.quaternion_from_euler(0, 0, face_target_yaw)

    # 6. 构建并返回最终的 Pose 对象
    goal_pose = [[goal_x, goal_y, 0.138], [q_tuple[0], q_tuple[1], q_tuple[2], q_tuple[3]]]
    return goal_pose
