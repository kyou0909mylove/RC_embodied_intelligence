# -*- coding: utf-8 -*-
"""按 jujia26.Controller 的同名方法发布初始先验；不是新的定位算法。"""
import time
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


class Controller:
    def publish_initial_pose(self):
        """沿用 jujia26 的准备等待、平面先验、协方差和两次发布；不复制旧坐标。"""
        pose, gate = self.initial_pose, self.deadline_gate
        if pose is None:
            raise ValueError("initial_pose 尚未填写")
        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.initial_pose_pub = pub
        # 原实现是固定等待；改为有界等订阅者，避免没有定位节点也报告成功。
        end = min(gate.deadline, gate.clock() + 5.0)
        while pub.get_num_connections() == 0:
            gate.check()
            if rospy.is_shutdown():
                raise RuntimeError("ROS 已停止")
            if gate.clock() >= end:
                raise RuntimeError("/initialpose 没有订阅者；请启动已有 AMCL/定位节点")
            time.sleep(0.05)
        # jujia26 等定位节点准备 5 秒；保留这个默认值，但不用可能冻结的 ROS sleep。
        end = gate.clock() + getattr(self, "initial_pose_wait_s", 5.0)
        while gate.clock() < end:
            gate.check()
            if rospy.is_shutdown():
                raise RuntimeError("ROS 已停止")
            time.sleep(min(0.05, max(0.0, end - gate.clock())))
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        p.x, p.y = pose[0][:2]
        p.z = 0.0  # 与 jujia26 一致：地面定位先验不使用旧导航点的 z=0.138。
        q.x, q.y, q.z, q.w = pose[1]
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        for index in range(2):
            if index:
                end = gate.clock() + 0.5
                while gate.clock() < end:
                    gate.check()
                    time.sleep(min(0.05, max(0.0, end - gate.clock())))
            gate.check()
            if rospy.is_shutdown():
                raise RuntimeError("ROS 已停止")
            if pub.get_num_connections() == 0:
                raise RuntimeError("/initialpose 订阅者已断开；未继续发布")
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
        rospy.loginfo("已发初始位姿请求；不代表定位收敛，发送入场目标前仍需人工确认")
