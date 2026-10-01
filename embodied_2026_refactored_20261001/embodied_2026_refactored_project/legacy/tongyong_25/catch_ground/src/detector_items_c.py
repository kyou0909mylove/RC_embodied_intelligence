#!/usr/bin/env python3
# ========== 中文阅读说明 ==========
# 这里包含相机封装和 Kinect 搜索检测器，是 main 的“看物品”模块。
# YoloResult 中 x/y 是图像像素；world() 反投影后才得到相机三维坐标。
# world() 虽名为 world，实际返回相机坐标，之后还须调用 camera_to_map 转成地图坐标。
# main 使用 KinectCamera 和 ItemsDetector.pred/world；其余相机与演示方法为原代码保留。
# main 从同一个 capture 取彩色与对齐深度，避免分别取帧造成时间错位。
# ==================================
"""
多相机目标检测封装"""

import numpy as np
import cv2
import torch
import time
from abc import ABC, abstractmethod
from ultralytics import YOLO
import pyrealsense2 as rs
import pykinect_azure as pykinect
from pykinect_azure import (
    K4A_CALIBRATION_TYPE_COLOR, 
    K4A_CALIBRATION_TYPE_DEPTH,
    K4A_FRAMES_PER_SECOND_30,
    K4A_WIRED_SYNC_MODE_STANDALONE
)
import sys
sys.path.append('/home/zq/catkin_ws/src/cmoon/src')
from base_controller import Base
import time

# 【类 YoloResult】
# 保存二维检测结果：类别 name、框 box、采样像素 x/y 和置信度 conf。
class YoloResult:
    # 【函数/方法 YoloResult.__init__】
    # 将检测字段保存起来；这里的 x/y 还不是米制三维坐标。
    def __init__(self, name, box, x, y, conf) -> None:
        self.name = name
        self.box = box
        self.x = x
        self.y = y
        self.conf = conf

    # 【函数/方法 YoloResult.__str__】
    # 把检测对象格式化为便于打印的字符串。
    def __str__(self):
        return f'name:{self.name},box:{self.box},x:{self.x},y:{self.y}'

# 【类 Camera】
# 抽象相机接口：规定取彩色、取深度、读内参和释放资源的方法名。
class Camera(ABC):
    """Abstract base class for all camera types"""
    
    # 【函数/方法 Camera.get_frame】
    # 抽象的彩色取帧接口，由具体相机类实现。
    @abstractmethod
    def get_frame(self):
        """Get color frame from camera"""
        pass
        
    # 【函数/方法 Camera.get_depth】
    # 抽象的深度取帧接口，由具体相机类实现。
    @abstractmethod
    def get_depth(self):
        """Get depth frame from camera"""
        pass
        
    # 【函数/方法 Camera.get_calibration】
    # 抽象的内参读取接口，具体相机返回 3×3 的 K。
    @abstractmethod
    def get_calibration(self):
        """Get camera calibration matrix"""
        pass
        
    # 【函数/方法 Camera.release】
    # 抽象的相机资源释放接口。
    @abstractmethod
    def release(self):
        """Release camera resources"""
        pass

# 【类 KinectCamera】
# Azure Kinect 的原 SDK 封装，当前 main 使用这一种相机。
class KinectCamera(Camera):
    """Implementation for Azure Kinect camera"""
    
    # 【函数/方法 KinectCamera.__init__】
    # 保存历史彩色内参 K；main 会用现场 JSON 的内参覆盖它。
    def __init__(self):
        self.K = np.array([915.0828247070312, 0.000000, 961.7936401367188,
                          0.000000, 914.6190185546875, 555.453369140625,
                          0.000000, 0.000000, 1.000000]).reshape(3,3)
    # 【函数/方法 KinectCamera.open_camera】
    # 初始化 SDK、设置彩色/深度模式，启动真实相机设备。
    def open_camera(self):
        pykinect.initialize_libraries()
        device_config = pykinect.default_configuration
        device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_MJPG
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P  # 1080P:1920x1080, 720P:1280x720
        device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
        device_config.camera_fps = K4A_FRAMES_PER_SECOND_30
        device_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE
        device_config.synchronized_images_only = True
        self.device = pykinect.start_device(config=device_config)

    # 【函数/方法 KinectCamera.get_frame】
    # 更新一次 capture 后取彩色图，返回 (是否成功,图像)。
    def get_frame(self):
        capture = self.device.update()
        ret, color_frame = capture.get_color_image()
        return ret, color_frame
        
    # 【函数/方法 KinectCamera.get_depth】
    # 另更新一次 capture 后取对齐深度，返回 (是否成功,深度图)。
    # 这与单独调用 get_frame 可能不是同一帧，所以 main 直接从同一个 capture 取两路图。
    def get_depth(self):
        capture = self.device.update()
        ret, depth_image = capture.get_transformed_depth_image()
        return ret, depth_image
        
    # 【函数/方法 KinectCamera.get_calibration】
    # 返回相机内参 K；这是像素投影参数，不是相机到机械臂的外参。
    def get_calibration(self):
        return self.K
        
    # 【函数/方法 KinectCamera.release】
    # 停止相机并关闭设备，释放 SDK 资源。
    def release(self):
        self.device.stop_cameras()
        self.device.close()

# 【类 RealSenseCamera】
# 另一种原相机封装，保留给旧模块使用，main 搜索没有用它。
class RealSenseCamera(Camera):
    """Implementation for Intel RealSense camera"""
    
    # 【函数/方法 RealSenseCamera.__init__】
    # 原初始化体为空，实际设备初始化在 open_camera。
    def __init__(self):
        pass
    
    # 【函数/方法 RealSenseCamera.open_camera】
    # 启动彩色/深度流，建立对齐对象，并从设备读取内参。
    def open_camera(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        
        profile = self.pipeline.get_active_profile()
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.K = np.array([intr.fx, 0, intr.ppx,
                          0, intr.fy, intr.ppy,
                          0, 0, 1]).reshape(3,3)
        
    # 【函数/方法 RealSenseCamera.get_frame】
    # 等待一批对齐帧，返回彩色图；失效返回 (False,None)。
    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        if not color_frame:
            return False, None
        return True, np.asanyarray(color_frame.get_data())
        
    # 【函数/方法 RealSenseCamera.get_depth】
    # 等待一批对齐帧，返回深度数组；它与另一次彩色读取不保证同帧。
    def get_depth(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame:
            return False, None
        return True, np.asanyarray(depth_frame.get_data())
        
    # 【函数/方法 RealSenseCamera.get_calibration】
    # 返回设备读取到的内参 K。
    def get_calibration(self):
        return self.K
        
    # 【函数/方法 RealSenseCamera.release】
    # 停止 RealSense 流。
    def release(self):
        self.pipeline.stop()

# 【类 WebCamera】
# 普通摄像头封装，没有深度能力；不是当前主流程相机。
class WebCamera(Camera):
    """Implementation for standard web camera"""
    
    # 【函数/方法 WebCamera.__init__】
    # 打开系统摄像头 0，并保存一个历史示例内参。
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.K = np.array([600, 0, 320,
                          0, 600, 240,
                          0, 0, 1]).reshape(3,3)
        
    # 【函数/方法 WebCamera.get_frame】
    # 返回普通摄像头 cap.read() 的成功标志和图像。
    def get_frame(self):
        ret, frame = self.cap.read()
        return ret, frame
        
    # 【函数/方法 WebCamera.get_depth】
    # 普通摄像头没有深度，因此固定返回 (False,None)。
    def get_depth(self):
        return False, None
        
    # 【函数/方法 WebCamera.get_calibration】
    # 返回保存的 K；这份历史示例不等于真实相机标定。
    def get_calibration(self):
        return self.K
        
    # 【函数/方法 WebCamera.release】
    # 关闭普通摄像头。
    def release(self):
        self.cap.release()

# 【类 ItemsDetector】
# 加载 YOLO，获得物品类别与图像中的二维位置。
class ItemsDetector:
    """Core detection class with YOLO model"""
    
    # 【函数/方法 ItemsDetector.__init__】
    # model_path 为模型文件；选 GPU/CPU 并加载 YOLO。
    # 统一 main 显式传入配置权重，不依赖这里的历史默认目录。
    def __init__(self, model_path='/home/zq/catkin_ws/src/cmoon/src/shijiazhuang_2025/tongyong_25/model/allbest.pt'):
        self.model = YOLO(model_path)
        print(f"内容：{self.model.names}")
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        print(f"Using device: {self.device}")

    # 【函数/方法 ItemsDetector.get_target_distance】
    # 按像素 x/y 读取深度，按本模块原约定从毫米换成米；无有效深度返回 None。
    def get_target_distance(self, depth_image, x, y):
        """获取目标距离，参考detect_people.py实现"""
        if depth_image is None:
            return None
        try:
            if 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]:
                distance = depth_image[int(y), int(x)] * 0.001
                return distance if distance > 0 else None
            return None
        except:
            return None

    # 【函数/方法 ItemsDetector.detect】
    # 旧连续检测循环：找到首个合格类别后返回 (True,三维点)，未找到返回 False 与零点。
    # 统一 main 用自己的有界扫描和 pred/world，没有调用这个旧循环。
    def detect(self, camera, target='person', max_distance=None, depth=True):
        """
        检测指定目标并返回其三维坐标，参考detect_people.py的返回格式
        参数:
            camera: 相机对象
            target: 目标物品名称
            max_distance: 最大检测距离(米)，None表示不限制
            depth: 是否使用深度信息
        返回:
            (has_target, 3d_coords)
            has_target: 布尔值，表示是否检测到目标
            3d_coords: 三维坐标元组(x, y, z)，若未检测到则为(0, 0, 0)
        """
        start_time = time.time()
        timeout = 8  # 超时时间(秒)
        
        while time.time() - start_time < timeout:
            ret, color_frame = camera.get_frame()
            if not ret:
                continue

            if depth:
                retd, depth_image = camera.get_depth()
                if not retd:
                    continue
            else:
                depth_image = None

            # 执行检测
            self.color_frame = color_frame
            yoloresults = self.pred()
            
            K = camera.get_calibration()
            height, width = color_frame.shape[:2]

            for result in yoloresults:
                # 检查是否是目标类别
                if result.name != target:
                    continue

                # 检查置信度
                if result.conf < 0.5:
                    continue

                # 检查是否在有效范围内
                if not self.judge_range(result.x, width, 1.0):  # 使用全范围检测
                    continue

                # 计算距离（如果需要）
                if depth and max_distance is not None:
                    distance = self.get_target_distance(depth_image, result.x, result.y)
                    if not distance or distance > max_distance:
                        continue
                elif depth:
                    distance = self.get_target_distance(depth_image, result.x, result.y)
                    if not distance:
                        continue
                else:
                    distance = 0.0  # 无深度信息时默认0

                # 计算三维坐标，与detect_people.py保持一致
                z = distance
                point_image = np.array([result.x, result.y, 1])
                point_3d = z * np.linalg.inv(K).dot(point_image)
                return (True, (point_3d[0], point_3d[1], point_3d[2]))

            if cv2.waitKey(10) in [ord('q'), 27]:
                break

        return (False, (0.0, 0.0, 0.0))
    
    # 【函数/方法 ItemsDetector.get_object_classes_sorted】
    # 从一帧中筛选合格框，按像素 x 从左到右输出类别名。
    # capitalize() 会改变标签大小写，因此 main 不用这个列表来匹配精确模型标签。
    def get_object_classes_sorted(self, camera, range=0.8, visualize=True):
        """
        获取所有识别到的物品种类，按从左到右顺序排列，支持可视化
        
        参数:
            camera: 相机设备对象
            range: 有效检测范围（0-1）
            visualize: 是否显示可视化结果
            
        返回:
            排序后的物品种类列表（字符串列表），按从左到右顺序排列
        """
        classes = []
        ret, color_frame = camera.get_frame()
        if not ret:
            return classes
        
        # 执行检测
        self.color_frame = color_frame.copy()
        yoloresults = self.pred()
        
        height, width = color_frame.shape[:2]
        
        # 筛选有效目标并收集信息
        valid_objects = []
        for result in yoloresults:
            if self.judge_range(result.x, width, range) and result.conf > 0.5:
                valid_objects.append({
                    'class': result.name,
                    'x': result.x,
                    'y': result.y,
                    'box': result.box,
                    'conf': result.conf
                })
        
        # 按x坐标（水平位置）从左到右排序
        valid_objects.sort(key=lambda x: x['x'])
        
        # 提取排序后的种类列表
        classes = [obj['class'].capitalize() for obj in valid_objects]
        
        # 可视化处理
        if visualize:
            self.visualize_sorted_objects(color_frame, valid_objects, width, range)
            
        return classes   
    
    # 【函数/方法 ItemsDetector.visualize_sorted_objects】
    # 画范围线、类别和编号，供旧检测界面调试；不发送机器人动作。
    def visualize_sorted_objects(self, frame, objects, width, range):
        """可视化排序后的目标对象"""
        # 绘制有效范围线
        height = frame.shape[0]
        cv2.line(frame, 
                (int(width * 0.5 * (1 - range)), 0),
                (int(width * 0.5 * (1 - range)), height),
                (0, 255, 0), 2)
        cv2.line(frame,
                (int(width * 0.5 * (1 + range)), 0),
                (int(width * 0.5 * (1 + range)), height),
                (0, 255, 0), 2)
        
        # 绘制每个目标
        for idx, obj in enumerate(objects):
            # 绘制边界框
            x1, y1, x2, y2 = obj['box']
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            
            # 绘制类别和置信度
            label = f"{obj['class']} ({obj['conf']:.2f})"
            cv2.putText(frame, label, (int(x1), int(y1)-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # 绘制排序编号（从左到右）
            cv2.putText(frame, f"#{idx+1}", (int(x1)+5, int(y1)+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        # 显示排序结果文本
        result_text = "排序: " + ", ".join([f"{i+1}.{obj['class']}" for i, obj in enumerate(objects)])
        cv2.putText(frame, result_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 保存图像到当前文件夹
        cv2.imwrite("sorted_objects_visualization.jpg", frame)
        
        # 显示图像并保持2秒后自动关闭
        cv2.imshow('Sorted Objects', frame)
        cv2.waitKey(2000)  # 等待2000毫秒（2秒）
        cv2.destroyWindow('Sorted Objects')
    
    # 【函数/方法 ItemsDetector.pred】
    # 对 self.color_frame 做 YOLO 推理，保存绘制框后的图，并返回 YoloResult 列表。
    # 结果为二维像素；Water 的采样位置有历史特殊处理。
    def pred(self):
        """Run YOLO prediction on frame"""
        results = self.model(self.color_frame)
        # plot()返回带检测框的图像；main保存这份图作为视觉证据。
        self.color_frame = results[0].plot()
        model_names = results[0].names
        
        yoloresults = []
        for result in results[0]:
            box = result.boxes
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = box.conf[0]
            cls = box.cls[0]
            if model_names[int(cls)] == "Water":
                center_y = y1/4 + y2/4*3
            else :
                center_y = (y1 + y2) / 2
            center_x = (x1 + x2) / 2
            yoloresult = YoloResult(model_names[int(cls)], [x1,y1,x2,y2], center_x, center_y, conf)
            yoloresults.append(yoloresult)
                
        return yoloresults
        
    # 【函数/方法 ItemsDetector.show】
    # 显示图像和横向有效范围线，属于调试 GUI。
    def show(self, frame, width, range):
        """Display frame with detection results"""
        height = frame.shape[0]
        cv2.line(frame, 
                (int(width * 0.5 * (1 - range)), 0),
                (int(width * 0.5 * (1 - range)), height),
                (0, 255, 0), 2, 4)
        cv2.line(frame,
                (int(width * 0.5 * (1 + range)), 0),
                (int(width * 0.5 * (1 + range)), height),
                (0, 255, 0), 2, 4)
        cv2.imshow('yolo', frame)
        
    # 【函数/方法 ItemsDetector.judge】
    # 按 pattern 判断是否退出、是否找到指定类别或是否存在范围内目标。
    def judge(self, pattern, yolors, target=None, resolution=640, range=0.8):
        """Determine if detection meets criteria"""
        if pattern == "realtime":
            return cv2.waitKey(1) & 0xFF == ord('q')
        elif pattern == 'find':
            for yolor in yolors:
                if target == yolor.name:
                    return (self.judge_range(yolor.x, resolution, range) 
                            and (yolor.conf > 0.5))
        else:
            for yolor in yolors:
                if self.judge_range(yolor.x, resolution, range):
                    return True
        return False
        
    # 【函数/方法 ItemsDetector.judge_range】
    # range 是画面中心横向比例，例如 0.8 表示中心 80% 宽度；不是空间距离。
    def judge_range(self, x, resolution, range):
        """Check if point is within target range"""
        left = resolution * 0.5 * (1 - range)
        right = resolution * 0.5 * (1 + range)
        return left <= x <= right
        
    # 【函数/方法 ItemsDetector.world】
    # 使用 P=深度×K的逆×[像素x,像素y,1]，把二维像素反投影为相机点。
    # 深度按原毫米约定乘 0.001；输出还需 TF 转成地图点。
    def world(self, camera, depth_image, yolors):
        """Calculate 3D world coordinates"""
        K = camera.get_calibration()
        x = yolors.x
        y = yolors.y
        z = depth_image[int(y), int(x)] * 0.001
        point_image = np.array([x, y, 1])
        # K的逆把像素射线还原成方向，再乘深度得到相机坐标点；尚未应用相机到map的TF。
        point = z * np.linalg.inv(K).dot(point_image)
        return point

# 【脚本入口】只有直接运行本文件才执行这里；从其它文件import不会进入这个分支。
# 统一工程从main_2026.py启动；旧模块末尾的历史演示不是经过主流程检查的比赛入口。
if __name__ == "__main__":
    # 初始化相机和检测器
    camera = KinectCamera()
    camera.open_camera()
    detector = ItemsDetector()
    
    # 示例：检测"cup"目标，最大距离5米
    has_target, coords = detector.detect(camera, target='cup', max_distance=5.0)
    print(f"是否检测到目标: {has_target}")
    if has_target:
        print(f"三维坐标: x={coords[0]:.2f}m, y={coords[1]:.2f}m, z={coords[2]:.2f}m")
    
    # 获取按从左到右排序的物品种类列表，并显示可视化结果
    object_classes = detector.get_object_classes_sorted(camera, range=0.8, visualize=True)
    print("识别到的物品（从左到右）:", object_classes)
    print("物品数量为:", len(object_classes))  
    
    camera.release()
    cv2.destroyAllWindows()
