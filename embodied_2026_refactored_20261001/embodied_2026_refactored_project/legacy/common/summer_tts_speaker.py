#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ========== 中文阅读说明 ==========
# 本文件把文字发给已有 SummerTTS 节点，真正的语音合成由外部节点完成。
# speak(text) 发布 String 消息；发布成功不等于已经播放完，也没有播放完成回执。
# main 在关键阶段检查订阅者连接，识别与交付记录仍保留各自的确认条件。
# ==================================

import rospy
from std_msgs.msg import String
"""
这是一个封装SummerTTS的ROS节点 用于发送文本到语音合成节点 触发语音播放
要先启动summer_tts_node节点 -> 位于主目录 文件名位summer_tts_ws 启动其中cpp节点 具体查阅文件内readme
speak为非阻塞时 节点发送信息后不会等待语音播放完成 在一些时候注意sleep
created by zx 2025-10-06
"""

# 【类 SummerTTSSpeaker】
# 把中文播报文本交给外部 SummerTTS 节点的封装。
class SummerTTSSpeaker:
    """封装SummerTTS的语音合成接口 提供speak方法发送文本到语音合成节点"""
    
    # 【函数/方法 SummerTTSSpeaker.__init__】
    # 创建 /summer_tts_topic 的 String 发布者，然后等待订阅者连接。
    def __init__(self):
        # 创建发布者，话题为/summer_tts_topic（与summer_tts_node订阅的话题一致）
        # 消息类型为std_msgs/String，队列大小自行调整
        self.tts_pub = rospy.Publisher(
            '/summer_tts_topic', 
            String, 
            queue_size=20
        )
        
        # 等待发布者与订阅者建立连接（避免第一条消息丢失）
        self._wait_for_subscribers()

    # 【函数/方法 SummerTTSSpeaker._wait_for_subscribers】
    # 按 ROS 时间最多等待约 5 秒；没有订阅者只记录警告。
    # 这里的返回不表示已经说话，main 还会独立检查连接数。
    def _wait_for_subscribers(self):
        """等待订阅者连接 最多等待5秒 避免程序卡死 """
        timeout = 5.0  # 超时时间（秒）
        start_time = rospy.get_time()
        
        while self.tts_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            if rospy.get_time() - start_time > timeout:
                rospy.logwarn("警告：未检测到/summer_tts_topic的订阅者（可能summer_tts_node未启动）")
                return
            rospy.sleep(0.1)  # 每100ms检查一次
        
        if self.tts_pub.get_num_connections() > 0:
            rospy.loginfo("已连接到语音合成节点，准备发送消息")

    # 【函数/方法 SummerTTSSpeaker.speak】
    # text 必须是字符串；构造消息后发布并记录日志。
    # 这是发送请求的方法，没有“播放完成”的成功返回值。
    def speak(self, text):
        """
        发送文本到语音合成节点，触发语音播放
        
        参数:
            text (str): 需要合成语音的文本（支持中文）
        """
        if rospy.is_shutdown():
            rospy.logerr("ROS节点已关闭，无法发送语音消息")
            return
        
        if not isinstance(text, str):
            rospy.logerr(f"输入文本必须是字符串，当前类型：{type(text)}")
            return
        
        # 创建消息并发布
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        rospy.loginfo(f"已发送语音文本：{text}")


# 测试代码
# 【脚本入口】只有直接运行本文件才执行这里；从其它文件import不会进入这个分支。
# 统一工程从main_2026.py启动；旧模块末尾的历史演示不是经过主流程检查的比赛入口。
if __name__ == '__main__':
    try:
        # 实例化语音合成接口
        speaker = SummerTTSSpeaker()
        
        # 测试发送多条消息
        speaker.speak("测试语音合成功能")
        rospy.sleep(2)  # 等待上一条语音播放完成
        #speaker.speak("Hello, Summer TTS in ROS")  #没有英文合成
        #rospy.sleep(2)
        speaker.speak("张翔是王吉吉爸爸")
        #speaker.speak("封装完成，可在主程序中直接调用speak函数")
        
        # 保持节点运行（实际主程序中可省略，由主程序管理节点生命周期）
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"发生错误：{str(e)}")
