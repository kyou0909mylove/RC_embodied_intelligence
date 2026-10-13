# -*- coding: utf-8 -*-
"""沿用往年realsense_yolo11.py的轮廓最小外接矩形角度，不用YOLO框宽高猜角度。"""
import math


def item_angle(image):
    import cv2
    import numpy as np
    if image.size==0:
        return None
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray,(5,5),0)
    edges = cv2.Canny(blurred,50,150)
    contours,_ = cv2.findContours(edges,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours if cv2.contourArea(c)>100]
    if not valid:
        return None  # 源代码返回0，无法区分测量失败和真实0度；这里明确拒绝无角度。
    rectangle = cv2.minAreaRect(np.vstack(valid))
    angle = rectangle[2]+(90 if rectangle[1][0]<rectangle[1][1] else 0)
    result = round(90-angle,1)
    return result if math.isfinite(result) else None


def angle_distance(a,b):
    """平躺物体主轴180度等价，避免+89/-89跨界被当成178度跳变。"""
    return abs((a-b+90)%180-90)
