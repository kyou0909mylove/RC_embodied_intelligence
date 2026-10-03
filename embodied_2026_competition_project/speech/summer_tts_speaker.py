#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import time
from std_msgs.msg import String
"""
这是一个封装SummerTTS的ROS节点 用于发送文本到语音合成节点 触发语音播放
要先启动summer_tts_node节点 -> 位于主目录 文件名位summer_tts_ws 启动其中cpp节点 具体查阅文件内readme
speak为非阻塞时 节点发送信息后不会等待语音播放完成 在一些时候注意sleep
created by zx 2025-10-06
"""

class SummerTTSSpeaker:
    """封装SummerTTS的语音合成接口 提供speak方法发送文本到语音合成节点"""
    
    def __init__(self):
        # ===== 2026-09-30 TTS播放完成同步 START =====
        self._last_done_text = None
        self._waiting_text = None
        self._done_lock = threading.Lock()
        self._done_event = threading.Event()
        # ===== 2026-09-30 TTS播放完成同步 END =====

        # 创建发布者，话题为/summer_tts_topic（与summer_tts_node订阅的话题一致）
        # 消息类型为std_msgs/String，队列大小自行调整
        self.tts_pub = rospy.Publisher(
            '/summer_tts_topic', 
            String, 
            queue_size=20
        )

        # ===== 2026-09-30 TTS播放完成同步 START =====
        # 修改后的 C++ summer_tts_node 会在真正播报完成后发布 /summer_tts_done。
        # 若现场仍是旧 C++ 节点，没有此 topic，也不会影响普通 speak，只是 wait_done 会超时。
        self.tts_done_sub = rospy.Subscriber(
            '/summer_tts_done',
            String,
            self._done_callback
        )
        # ===== 2026-09-30 TTS播放完成同步 END =====
        
        # 等待发布者与订阅者建立连接（避免第一条消息丢失）
        self._wait_for_subscribers()

    def _wait_for_subscribers(self):
        """等待订阅者连接 最多等待5秒 避免程序卡死 """
        timeout = 5.0  # 超时时间（秒）
        start_time = time.monotonic()
        
        while self.tts_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            gate = getattr(self, "deadline_gate", None)
            if gate is not None:
                gate.check()
            if time.monotonic() - start_time >= timeout:
                rospy.logwarn("警告：未检测到/summer_tts_topic的订阅者（可能summer_tts_node未启动）")
                return
            time.sleep(0.05)  # 使用单调时钟，不因 ROS /clock 停止而无限等待。
        
        if self.tts_pub.get_num_connections() > 0:
            rospy.loginfo("已连接到语音合成节点，准备发送消息")

    # ===== 2026-09-27 语音联调诊断修改 START =====
    # 用于 voice_speaker_demo_2026.py 在测试前判断 SummerTTS 节点是否真的已连接。
    # 注意：rospy 打印“已发送语音文本”只代表消息发布到 topic，不代表已有 TTS 节点订阅并播放。
    def is_connected(self):
        """返回 /summer_tts_topic 当前是否存在订阅者。"""
        return self.tts_pub.get_num_connections() > 0
    # ===== 2026-09-27 语音联调诊断修改 END =====

    # ===== 2026-09-30 TTS播放完成同步 START =====
    def _done_callback(self, msg):
        """收到 C++ 节点播报完成信号；只唤醒当前正在等待的那一句，避免旧 done 串扰。"""
        if str(getattr(msg, "_connection_header", {}).get("latching", "0")) == "1":
            return  # 不接受锁存的历史完成消息。
        with self._done_lock:
            self._last_done_text = msg.data
            # 没有等待指定文本时仅记录；有等待文本时必须文本一致才唤醒。
            if self._waiting_text is None:
                return
            if msg.data == self._waiting_text:
                self._done_event.set()
            else:
                rospy.logwarn(
                    f"收到非当前播报的 /summer_tts_done，已忽略：done={msg.data}，waiting={self._waiting_text}"
                )
    # ===== 2026-09-30 TTS播放完成同步 END =====

    def speak(self, text, wait_done=False, timeout=30.0):
        """
        发送文本到语音合成节点，触发语音播放
        
        参数:
            text (str): 需要合成语音的文本（支持中文）
            wait_done (bool): 是否等待 /summer_tts_done 播报完成信号
            timeout (float): 等待完成信号的最大秒数
        """
        if rospy.is_shutdown():
            rospy.logerr("ROS节点已关闭，无法发送语音消息")
            return False
        
        if not isinstance(text, str):
            rospy.logerr(f"输入文本必须是字符串，当前类型：{type(text)}")
            return False
        gate = getattr(self, "deadline_gate", None)
        if gate is not None:
            gate.check()
        if not self.is_connected():
            rospy.logwarn("/summer_tts_topic 没有订阅者；未发送播报请求")
            return False
        
        # 创建消息并发布
        msg = String()
        msg.data = text
        # 等待完成前必须先清除上一次 done，并记录当前等待文本，避免收到旧 done 后提前录音。
        if wait_done:
            with self._done_lock:
                self._done_event.clear()
                self._last_done_text = None
                self._waiting_text = text
        self.tts_pub.publish(msg)
        # ===== 2026-09-27 语音联调诊断修改 START =====
        if not self.is_connected():
            rospy.logwarn(
                "已发布语音文本，但 /summer_tts_topic 当前没有订阅者，"
                "不会真正播报。请先启动 summer_tts_node，或检查话题名是否一致。"
            )
            if wait_done:
                with self._done_lock:
                    self._waiting_text = None
                return False
        # ===== 2026-09-27 语音联调诊断修改 END =====
        rospy.loginfo(f"已发送语音文本：{text}")

        # ===== 2026-09-30 TTS播放完成同步 START =====
        if wait_done:
            rospy.loginfo(f"等待 /summer_tts_done 播报完成信号，目标文本：{text}")
            end = time.monotonic() + timeout
            try:
                while True:
                    if gate is not None:
                        gate.check()
                    if rospy.is_shutdown():
                        return False
                    # 迟到的 done 不能突破阶段或比赛截止。
                    if time.monotonic() >= end:
                        ok = False
                        break
                    if self._done_event.is_set():
                        ok = True
                        break
                    time.sleep(min(0.05, max(0.0, end - time.monotonic())))
            finally:
                with self._done_lock:
                    last_done = self._last_done_text
                    self._waiting_text = None
            if ok:
                rospy.loginfo(f"收到匹配的播报完成信号：{last_done}")
                return True
            rospy.logwarn("等待 /summer_tts_done 超时；没有把估算等待当成播报完成")
            return False
        return True
        # ===== 2026-09-30 TTS播放完成同步 END =====


# 原类接口保持。由 main/testing 初始化 ROS 后使用，不携带无关旧演示。
