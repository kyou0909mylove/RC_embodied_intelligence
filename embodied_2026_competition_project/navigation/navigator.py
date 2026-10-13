#!/usr/bin/env python3
# !coding=utf-8
# ========== 中文阅读说明 ==========
# Navigator 把目标位姿交给已有 move_base 导航服务，不重新实现路径规划。
# main 使用 __init__、set_goal、client；取消采用当前GoalID，不调用旧stop。
# goto/go_to_location 是旧的演示流程，含反复清图和重试，统一 main 没有调用。
# deadline_gate 是 main 注入的截止门控；有它时构造器等待会受到时间限制。
# ==================================
# Created by Cmoon - Optimized Version by zx 2025-10-10

import rospy
from std_srvs.srv import Empty
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from core.faults import DeviceUnavailable


# 【类 Navigator】
# 导航接口对象：保存点位、构造导航消息并访问 move_base 客户端。
class Navigator:
    # 【函数/方法 Navigator.__init__】
    # location 是“地点名称→位姿列表”的字典。
    # 连接动作服务器与清图服务；有 main 注入的门控时限制等待时长。
    def __init__(self, location):
        """
        location是字典,键是地点名字(String),值是坐标列表
        例:'door': [[-4.35, -6.18, 0.0], [0.0, 0.0, -0.20, -0.97]]
        """
        self.location = location
        self.goal = MoveBaseGoal()  # 实例化MoveBaseGoal消息类型
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        gate = getattr(self, 'deadline_gate', None)
        if gate is None:
            self.client.wait_for_server()
        elif not self.client.wait_for_server(rospy.Duration(min(getattr(self,'server_timeout_s',5.0),gate.check()))):
            raise DeviceUnavailable('导航','move_base action server 持续不可达')
        if gate is not None:
            gate.check()
        rospy.loginfo("Move_base action server connected.")
        # 定义清理代价地图的服务客户端
        self.clear_costmap_client = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        clear_connected = True
        if gate is None:
            rospy.wait_for_service('move_base/clear_costmaps')
        else:
            try:
                rospy.wait_for_service('move_base/clear_costmaps',timeout=min(getattr(self,'server_timeout_s',5.0),gate.check()))
            except rospy.ROSException as exc:
                # 清图是辅助服务，缺少它不等于move_base不可用；目标导航仍可继续。
                rospy.logwarn('clear_costmaps暂不可用，继续当前导航：'+str(exc))
                clear_connected = False
            gate.check()
        if clear_connected:
            rospy.loginfo("Clear_costmaps service connected.")

        rospy.loginfo('Navigator is ready.')

    # 【函数/方法 Navigator.goto】
    # 旧便捷方法：用地点名称取配置位姿，再调用旧 go_to_location。
    # 统一 main 不调用它，因为下面的旧导航重试不是限次流程。
    def goto(self, place):
        """根据预设的地点名称进行导航"""
        if place not in self.location:
            rospy.logerr(f"Error: Location '{place}' not found in the dictionary.")
            return

        point = self.set_goal("map", self.location[place][0], self.location[place][1])
        self.go_to_location(point)
        rospy.loginfo(f"Successfully arrived at {place}.")

    # 【函数/方法 Navigator.set_goal】
    # 把 frame_id、position=[x,y,z]、orientation=[qx,qy,qz,qw] 写入 MoveBaseGoal。
    # 只创建/更新消息；真正发送由 client.send_goal 完成。
    def set_goal(self, frame_id, position, orientation):
        """设置导航目标点的坐标和姿态"""
        self.goal.target_pose.header.frame_id = frame_id
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = position[2]
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]
        return self.goal

    # 【函数/方法 Navigator.go_to_location】
    # 历史重试循环：清代价地图、发目标、等待，失败后再次尝试。
    # 该方法本身没有次数上限，当前 main 使用自己的有界流程。
    def go_to_location(self, location_goal):
        flag = False
        while not flag and not rospy.is_shutdown():  # 导航重试循环
            rospy.loginfo("Attempting to navigate...")
            self.clear_costmap_client()  # 每次尝试前清理代价地图

            self.client.send_goal(location_goal)
            self.client.wait_for_result()

            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation successful!")
                flag = True
            else:
                rospy.logwarn("Navigation failed. Retrying...")
                rospy.sleep(1)  # 短暂等待后重试

    # 【函数/方法 Navigator.stop】
    # 发送取消导航请求，不创建新的移动目标。
    def stop(self):
        """停止当前的导航任务"""
        rospy.loginfo("Cancelling all navigation goals.")
        self.client.cancel_all_goals()
