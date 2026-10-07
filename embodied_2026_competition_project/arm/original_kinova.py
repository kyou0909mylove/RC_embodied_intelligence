# -*- coding: utf-8 -*-
"""Kinova 驱动：来自 kinova_wn/catch_ground/src/catch.py。

原动作发送、米/度/百分比转换保持来源实现。这里只保留 main 使用的驱动；
观察、坐标计算、抓取和投放分别位于独立模块，不再加载旧业务相机和语音。
ROS 节点由 core/session.py 创建，新场次复位由 arm/startup.py 执行。
"""
import roslib
roslib.load_manifest('kinova_demo')
import rospy
import math
import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg


class KinovaRobot:
    """使用已有机械臂 ROS 驱动；不启动第二个 ROS 节点。"""
    def __init__(self,kinova_robotType, initialize_motion=True, feedback_timeout_s=10.0, server_timeout_s=5.0, arm_timeout_s=20.0, finger_timeout_s=10.0) -> None:
        rospy.loginfo("Kinova 初始化：使用主程序已创建的 ROS 节点")
        self.feedback_timeout_s = float(feedback_timeout_s)
        self.server_timeout_s = float(server_timeout_s)
        self.arm_timeout_s = float(arm_timeout_s)
        self.finger_timeout_s = float(finger_timeout_s)
        if not math.isfinite(self.feedback_timeout_s) or self.feedback_timeout_s <= 0:
            raise ValueError('initial_feedback_timeout_s 须为有限正秒数')
        self.kinova_robotType = kinova_robotType
        self.prefix = self.kinova_robotType + "_"
        self.arm_joint_number = int(kinova_robotType[3])
        self.finger_number = int(kinova_robotType[5])
        self.finger_maxDist = 18.9/2/1000
        self.finger_maxTurn = 6800
        self.currentFingerPosition = [0.0, 0.0, 0.0]
        self.currentCartesianCommand = [0.2104809731245041, -0.25873029232025146, 0.5095799565315247, 1.6373136043548584, 1.1021580696105957, 0.5095799565315247]
        self.homePositionMdeg = [0.2104809731245041, -0.25873029232025146, 0.5095799565315247, 81.040, 83.972, 11.606]
        self.getcurrentCartesianCommand()
        self.action_address_arm = '/' + self.prefix + 'driver/pose_action/tool_pose'
        self.client_arm = actionlib.SimpleActionClient(self.action_address_arm, kinova_msgs.msg.ArmPoseAction)
        if not self.client_arm.wait_for_server(rospy.Duration(self.server_timeout_s)):  # 超时5秒
            rospy.logerr("Arm action server not available!")
            rospy.signal_shutdown("Arm action server not found")
            return
        self.goal_arm = kinova_msgs.msg.ArmPoseGoal()
        self.goal_arm.pose.header = std_msgs.msg.Header(frame_id=(self.prefix + 'link_base'))
        rospy.loginfo("arm service connect successfully")

        self.getCurrentFingerPosition()
        self.action_address_finger = '/' + self.prefix + 'driver/fingers_action/finger_positions'
        self.client_finger = actionlib.SimpleActionClient(self.action_address_finger, kinova_msgs.msg.SetFingersPositionAction)
        if not self.client_finger.wait_for_server(rospy.Duration(self.server_timeout_s)):
            rospy.logerr("Finger action server not available!")
            rospy.signal_shutdown("Finger action server not found")
            return
        self.goal_finger = kinova_msgs.msg.SetFingersPositionGoal()
        rospy.loginfo("finger service connect successfully")
        if initialize_motion:
            self.finger_run(finger_target=[5,5,5])
        self.speak = None  # 语音由主程序统一准备，驱动不另建播报器。


        if initialize_motion:
            self.arm_run(pose_target=self.homePositionMdeg)

    def arm_run(self,unit='mdeg',pose_target=None,relative=False):
        pose_mq, pose_mdeg, pose_mrad = self.unitParser_arm(unit,pose_target,relative)
        try:
            self.poses = [float(n) for n in pose_mq]
            self.cartesian_pose_client(self.poses[:3], self.poses[3:])
            print('Cartesian pose sent!')
        except rospy.ROSInterruptException:
            print("program interrupted before completion")

    def finger_run(self,unit='percent',finger_target=None,relative=False):
        finger_turn, finger_meter, finger_percent = self.unitParser_finger(unit, finger_target, relative)
        try:
            if self.finger_number == 0:
                print('Finger number is 0, check with "-h" to see how to use this node.')
                self.positions = []  # Get rid of static analysis warning that doesn't see the exit()
                exit()
            else:
                positions_temp1 = [max(0.0, n) for n in finger_turn]
                positions_temp2 = [min(n, self.finger_maxTurn) for n in positions_temp1]
                self.positions = [float(n) for n in positions_temp2]

            print('Sending finger position ...')
            result = self.gripper_client(self.positions)
            print('Finger position sent!')

        except rospy.ROSInterruptException:
            print('program interrupted before completion')

    def cartesian_pose_client(self, position, orientation):
        """Send a cartesian goal to the action server."""
        self.goal_arm.pose.pose.position = geometry_msgs.msg.Point(
            x=position[0], y=position[1], z=position[2])
        self.goal_arm.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

        # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

        self.client_arm.send_goal(self.goal_arm)

        if self.client_arm.wait_for_result(rospy.Duration(self.arm_timeout_s)):
            return self.client_arm.get_result()
        else:
            self.client_arm.cancel_all_goals()
            print('the cartesian action timed-out')
            return None

    def gripper_client(self, finger_positions):
        """Send a gripper goal to the action server."""
        self.goal_finger.fingers.finger1 = float(finger_positions[0])
        self.goal_finger.fingers.finger2 = float(finger_positions[1])
        # The MICO arm has only two fingers, but the same action definition is used
        if len(finger_positions) < 3:
            self.goal_finger.fingers.finger3 = 0.0
        else:
            self.goal_finger.fingers.finger3 = float(finger_positions[2])
        self.client_finger.send_goal(self.goal_finger)

        # 如果等待结果超时，会取消所有未完成的目标
        if self.client_finger.wait_for_result(rospy.Duration(self.finger_timeout_s)):
            return self.client_finger.get_result()
        else:
            self.client_finger.cancel_all_goals()
            rospy.logwarn('the gripper action timed-out')
            return None

    def unitParser_arm(self, unit_, pose_value_, relative_):
        """ Argument unit """
        position_ = pose_value_[:3]
        orientation_ = pose_value_[3:]
        print(f"position_:{position_}, orientation_:{orientation_}")

        for i in range(0,3):
            if relative_:
                position_[i] = pose_value_[i] + self.currentCartesianCommand[i]
            else:
                position_[i] = pose_value_[i]

        # print('pose_value_ in unitParser 1: {}'.format(pose_value_))  # debug

        if unit_ == 'mq':
            # 四元数
            if relative_:
                orientation_XYZ = self.Quaternion2EulerXYZ(orientation_)
                orientation_xyz_list = [orientation_XYZ[i] + self.currentCartesianCommand[3+i] for i in range(0,3)]
                orientation_q = self.EulerXYZ2Quaternion(orientation_xyz_list)
            else:
                orientation_q = orientation_

            orientation_rad = self.Quaternion2EulerXYZ(orientation_q)
            orientation_deg = list(map(math.degrees, orientation_rad))

        elif unit_ == 'mdeg':
            # 角度欧拉角
            if relative_:
                orientation_deg_list = list(map(math.degrees, self.currentCartesianCommand[3:]))
                orientation_deg = [orientation_[i] + orientation_deg_list[i] for i in range(0,3)]
            else:
                orientation_deg = orientation_

            orientation_rad = list(map(math.radians, orientation_deg))
            orientation_q = self.EulerXYZ2Quaternion(orientation_rad)

        elif unit_ == 'mrad':
            # 弧度欧拉角
            if relative_:
                orientation_rad_list =  self.currentCartesianCommand[3:]
                orientation_rad = [orientation_[i] + orientation_rad_list[i] for i in range(0,3)]
            else:
                orientation_rad = orientation_

            orientation_deg = list(map(math.degrees, orientation_rad))
            orientation_q = self.EulerXYZ2Quaternion(orientation_rad)

        else:
            raise Exception("Cartesian value have to be in unit: mq, mdeg or mrad")

        pose_mq_ = position_ + orientation_q
        pose_mdeg_ = position_ + orientation_deg
        pose_mrad_ = position_ + orientation_rad

        # print('pose_mq in unitParser 1: {}'.format(pose_mq_))  # debug

        return pose_mq_, pose_mdeg_, pose_mrad_

    def QuaternionNorm(self, Q_raw):
        qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
        qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
        qx_ = qx_temp/qnorm
        qy_ = qy_temp/qnorm
        qz_ = qz_temp/qnorm
        qw_ = qw_temp/qnorm
        Q_normed_ = [qx_, qy_, qz_, qw_]
        return Q_normed_

    def Quaternion2EulerXYZ(self, Q_raw):
        Q_normed = self.QuaternionNorm(Q_raw)
        qx_ = Q_normed[0]
        qy_ = Q_normed[1]
        qz_ = Q_normed[2]
        qw_ = Q_normed[3]

        tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
        ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
        tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
        EulerXYZ_ = [tx_,ty_,tz_]
        return EulerXYZ_

    def EulerXYZ2Quaternion(self, EulerXYZ_):
        print("EulerXYZ_:",EulerXYZ_)
        tx_, ty_, tz_ = EulerXYZ_[0:3]
        sx = math.sin(0.5 * tx_)
        cx = math.cos(0.5 * tx_)
        sy = math.sin(0.5 * ty_)
        cy = math.cos(0.5 * ty_)
        sz = math.sin(0.5 * tz_)
        cz = math.cos(0.5 * tz_)

        qx_ = sx * cy * cz + cx * sy * sz
        qy_ = -sx * cy * sz + cx * sy * cz
        qz_ = sx * sy * cz + cx * cy * sz
        qw_ = -sx * sy * sz + cx * cy * cz

        Q_ = [qx_, qy_, qz_, qw_]
        return Q_

    def unitParser_finger(self, unit_, finger_value_, relative_):
        """ Argument unit """
        # 根据用户指定的单位将目标值转换为内部表示
        # transform between units
        if unit_ == 'turn':
            # get absolute value
            if relative_:
                finger_turn_absolute_ = [finger_value_[i] + self.currentFingerPosition[i] for i in range(0, len(finger_value_))]
            else:
                finger_turn_absolute_ = finger_value_

            finger_turn_ = finger_turn_absolute_
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]

        elif unit_ == 'mm':
            # get absolute value
            finger_turn_command = [x/1000 * self.finger_maxTurn / self.finger_maxDist for x in finger_value_]
            if relative_:
                finger_turn_absolute_ = [finger_turn_command[i] + self.currentFingerPosition[i] for i in range(0, len(finger_value_))]
            else:
                finger_turn_absolute_ = finger_turn_command

            finger_turn_ = finger_turn_absolute_
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]
        elif unit_ == 'percent':
            # get absolute value
            finger_turn_command = [x/100.0 * self.finger_maxTurn for x in finger_value_]
            if relative_:
                finger_turn_absolute_ = [finger_turn_command[i] + self.currentFingerPosition[i] for i in
                                        range(0, len(finger_value_))]
            else:
                finger_turn_absolute_ = finger_turn_command

            finger_turn_ = finger_turn_absolute_
            finger_meter_ = [x * self.finger_maxDist / self.finger_maxTurn for x in finger_turn_]
            finger_percent_ = [x / self.finger_maxTurn * 100.0 for x in finger_turn_]
        else:
            raise Exception("Finger value have to be in turn, mm or percent")

        return finger_turn_, finger_meter_, finger_percent_

    def _wait_initial_feedback(self, topic_address, message_type, description):
        """驱动未发布反馈时给出具体话题，避免初始化一直停在一行日志。"""
        rospy.loginfo('等待%s：%s（最多 %.1f 秒）',
                      description, topic_address, self.feedback_timeout_s)
        try:
            return rospy.wait_for_message(topic_address, message_type,
                                          timeout=self.feedback_timeout_s)
        except rospy.ROSException as exc:
            raise RuntimeError(
                '{}等待失败：{}；请检查 kinova_bringup 驱动及两个终端的 ROS 网络环境。'
                .format(description, topic_address)) from exc

    def getcurrentCartesianCommand(self):
        # wait to get current position
        topic_address = '/' + self.prefix + 'driver/out/cartesian_command'
        self._cartesian_sub = rospy.Subscriber(topic_address, kinova_msgs.msg.KinovaPose, self.setcurrentCartesianCommand)
        self._wait_initial_feedback(topic_address, kinova_msgs.msg.KinovaPose, '机械臂位姿反馈')
        print('position listener obtained message for Cartesian pose. ')

    def setcurrentCartesianCommand(self, feedback):
        # print("-------------111------------")
        currentCartesianCommand_str_list = str(feedback).split("\n")
        # print(currentCartesianCommand_str_list)
        for index in range(0,len(currentCartesianCommand_str_list)):
            temp_str=currentCartesianCommand_str_list[index].split(": ")
            # print("temp_str:",temp_str)
            self.currentCartesianCommand[index] = float(temp_str[1])

    def getCurrentFingerPosition(self):
        # wait to get current position
        # 获取当前夹爪手指的位置
        topic_address = '/' + self.prefix + 'driver/out/finger_position'
        self._finger_sub = rospy.Subscriber(topic_address, kinova_msgs.msg.FingerPosition, self.setCurrentFingerPosition) # 将接收到的手指位置通过setCurrentFingerPosition存入全局变量
        self._wait_initial_feedback(topic_address, kinova_msgs.msg.FingerPosition, '夹爪位置反馈')
        print('obtained current finger position ')

    def setCurrentFingerPosition(self,feedback):
        self.currentFingerPosition[0] = feedback.finger1
        self.currentFingerPosition[1] = feedback.finger2
        self.currentFingerPosition[2] = feedback.finger3
