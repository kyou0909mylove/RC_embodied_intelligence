#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""复用原打点脚本；支持 AMCL 带 map 帧消息，打印平面位姿，不发送运动指令。"""
import argparse
import json
import math
import sys
import time
from threading import Event


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/robot_pose")
    parser.add_argument("--type", choices=("pose", "pose_stamped", "pose_covariance"),
                        default="pose", help="必须与 rostopic type 的结果一致")
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 60:
        parser.error("--timeout 须为 (0,60] 秒")
    import rospy
    from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
    ready, received = Event(), []

    def callback(message):
        if not ready.is_set():
            received.append(message)
            ready.set()

    rospy.init_node("record_navigation_pose", anonymous=True)
    message_type = {"pose": Pose, "pose_stamped": PoseStamped,
                    "pose_covariance": PoseWithCovarianceStamped}[args.type]
    subscriber = rospy.Subscriber(args.topic, message_type, callback, queue_size=1)
    end = time.monotonic() + args.timeout
    try:
        while not ready.is_set() and not rospy.is_shutdown() and time.monotonic() < end:
            ready.wait(0.05)
        if not received:
            print("未收到位姿，请检查 ROS 网络与 /robot_pose 发布节点")
            return 1
        message = received[0]
        if args.type == "pose":
            print("提示：Pose 消息没有 frame_id，须确认 /robot_pose 本来就是 map 位姿，不能用 odom 代替。",
                  file=sys.stderr)
            pose = message
        else:
            if message.header.frame_id.lstrip("/") != "map":
                raise ValueError("收到的位姿不是 map 坐标；本脚本不自动转换坐标")
            pose = message.pose.pose if args.type == "pose_covariance" else message.pose
        p, q = pose.position, pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            raise ValueError("位姿包含非有限数值")
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if abs(norm - 1.0) > 0.02:
            raise ValueError("位姿四元数未归一化")
        qx, qy, qz, qw = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))
        result = [[p.x, p.y, 0.0], [0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2)]]
        print(json.dumps(result, ensure_ascii=False))
        return 0
    finally:
        subscriber.unregister()


if __name__ == "__main__":
    raise SystemExit(main())
