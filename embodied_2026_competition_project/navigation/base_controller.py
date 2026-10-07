#!/usr/bin/env python
# coding: UTF-8 
# ========== 中文阅读说明 ==========
# Base 通过 /cmd_vel 发布底盘速度，通过 /robot_pose 接收当前位姿。
# 统一 main 主要使用 Base 的初始化和 stop，不用这里的旧 turn/rotate 自己导航。
# Twist 是速度消息：linear 为平移速度，angular 为旋转速度。
# 这些方法会直接控制底盘；文件末尾的旧演示也可能发送动作。
# ==================================

import rospy
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry
import math
import time


# 【类 Base】
# 底盘速度发布和位姿接收对象；统一主流程用 stop 发送零速度。
class Base:
    # 【函数/方法 Base.__init__】
    # 创建速度发布者并订阅位姿，初始化位置、四元数和 Twist 对象。
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0
        self.ox = 0.0
        self.oy = 0.0
        self.oz = 0.0
        self.ow = 0.0
        self.position = []  # 坐标
        self.orientation = []  # 四元数
        self.angle = 0  # 角度
        self.pose_sub = rospy.Subscriber('/robot_pose', Pose, self.now_pose)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            gate = getattr(self, "deadline_gate", None)
            if gate is not None:
                gate.check()
            if rospy.is_shutdown():
                raise RuntimeError("ROS 已停止")
            time.sleep(0.05)

    # 【函数/方法 Base.now_pose】
    # ROS 收到 /robot_pose 后调用的回调，把消息字段保存到实例属性。
    # 这里的 self 表示当前 Base 对象，不是另一个机器人。
    def now_pose(self, pose):
        """实时更新现在的坐标和四元数和角度"""
        self.px = pose.position.x
        self.py = pose.position.y
        self.pz = pose.position.z
        self.ox = pose.orientation.x
        self.oy = pose.orientation.y
        self.oz = pose.orientation.z
        self.ow = pose.orientation.w
        self.position = [self.px, self.py, self.pz]
        self.orientation = [self.ox, self.oy, self.oz, self.ow]
        self.get_angle()

    # 【函数/方法 Base.get_pose】
    # 返回当前保存的 [[位置],[四元数]]，没有额外请求一次新定位。
    def get_pose(self):
        """可调用获取现在的坐标和四元数"""
        print('[[{},{},{}],[{},{},{},{}]]'.format(self.px, self.py, self.pz, self.ox, self.oy, self.oz, self.ow))
        return [[self.px, self.py, self.pz], [self.ox, self.oy, self.oz, self.ow]]

    # 【函数/方法 Base.get_angle】
    # 将保存的四元数转换为平面朝向角，单位为弧度。
    def get_angle(self):
        """可调用获取现在的角度"""
        eular = self.quad2euler(self.ox, self.oy, self.oz, self.ow)
        self.angle = eular
        return eular

    # 【函数/方法 Base.turn】
    # 旧转向方法：输入 angle 是角度，内部换成弧度并按当前误差发转速。
    # 历史签名保留 kd 参数，当前方法体实际没有使用它；统一 main 不调用此闭环。
    def turn(self, angle, kp=1.5, kd=0.5):
        """传入转的角度,单位°,正值左转,负值右转,范围0~180°"""
        print('turing')
        # angle输入为角度；除以180再乘π，转为后续控制计算所需的弧度。
        angle_radian = (float(angle) / 180) * math.pi
        start_angle = self.get_angle()
        end_angle = start_angle + angle_radian
        if end_angle > math.pi:
            end_angle = end_angle - 2 * math.pi
        elif end_angle < -math.pi:
            end_angle = end_angle + 2 * math.pi
        error = angle_radian
        rate = rospy.Rate(1000)
        print(angle_radian)
        print(self.angle)
        print(start_angle)
        print(end_angle)
        print(self.angle - end_angle)
        while abs(self.angle - end_angle) > 0.02:
            last_error = error
            error = abs(self.angle - end_angle)
            if error > math.pi:
                error = 2 * math.pi - error
            if angle >= 0:
                speed = kp * (error + 0.01)
            else:
                speed = -kp * (error + 0.01)
            print('{},{}'.format(error, speed))
            self.rotate(speed)
            rate.sleep()
        self.stop()

    # 【函数/方法 Base.rotate】
    # 向 /cmd_vel 发布纯旋转速度，平移速度设为零。
    # speed 写入 angular.z，按 ROS 速度消息约定使用弧度/秒。
    def rotate(self, speed):
        """旋转"""
        print('rotating')
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = speed
        self.pub.publish(self.twist)

    # 【函数/方法 Base.stop】
    # 将 Twist 的全部平移和旋转分量设为零后发布。
    # 它是停止请求，不能只凭调用返回判断真实底盘已静止。
    def stop(self):
        print('stop')
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.pub.publish(self.twist)

    # 【函数/方法 Base.quad2euler】
    # 输入四元数 x/y/z/w，计算机器人绕竖直轴的 yaw 弧度。
    def quad2euler(self, x, y, z, w):
        X = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        Y = math.asin(2 * (w * y - x * z))
        Z = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
        return Z
