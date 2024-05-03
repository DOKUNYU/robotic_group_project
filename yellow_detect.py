#!/usr/bin/env python3
# 彩色帧率 = 深度帧率 = 58fps  
import numpy as np
import cv2


# 识别黄色矩形，提取角点
def yellow_detect(img):
    lower_yellow = np.array([22, 93, 0])
    upper_yellow = np.array([45, 255, 255])
    hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    thresh = cv2.inRange(hsv_img,lower_yellow,upper_yellow)
    kernel = np.ones((3,3),np.uint8)
    thresh = cv2.GaussianBlur(thresh,(5,5),20,20)
    dilate = cv2.dilate(thresh,kernel)
    contours, _ = cv2.findContours(dilate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for i in range(len(contours)):
        approx = cv2.approxPolyDP(contours[i], 0.1 * cv2.arcLength(contours[i], True), True) # 多边形逼近
        cv2.drawContours(img, [approx], 0, (255, 255, 0), 1) 
        pixel_left_up = approx[0][0]
        pixel_right_up = approx[3][0]
        pixel_right_down = approx[2][0]
        pixel_left_down = approx[1][0] 
        cv2.circle(img, pixel_left_up, 5, [0,0,255], thickness=-1) # 左上 红色
        cv2.circle(img, pixel_right_up, 5, [0,255,0], thickness=-1) # 右上 绿色
        cv2.circle(img, pixel_right_down,5, [255,0,0], thickness=-1) # 右下 蓝色
        cv2.circle(img, pixel_left_down, 5, [255,255,255], thickness=-1) # 左下 白色
        return pixel_left_up,pixel_right_up,pixel_right_down,pixel_left_down        
