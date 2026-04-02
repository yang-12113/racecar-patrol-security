import cv2
# import rclpy
import numpy as np
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

def create_mask(picture, mask_point):
    """ 创建掩模函数 """
    mask = np.zeros_like(picture)
    cv2.fillPoly(mask, mask_point, 255)
    mask_img = cv2.bitwise_and(picture, mask)
    return mask_img

def calculate_k_and_b(contour):
    """ 根据轮廓计算斜率 k 和截距 b """
    rows, cols = [480, 640]
    [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
    k = vy / vx
    b = y - k * x
    return k, b

def angle_calculate(k, b, frame):

    """ 根据斜率和截距计算角度 """
    # if K <= 0:
    #     angle =90 + K / 90
    # else:
    k = 2 * k
    angle = 90 + 90 / k
    if angle < 50:
        angle = 50
    if angle > 120:
        angle =120
    return angle

def process_frame(frame, lower_yellow, upper_yellow):
    """ 处理每一帧图像以检测黄色车道线和计算角度 """
    mask_point = np.array([[(0, 0), (640, 0), (640, 480), (0, 480)]])  # 道路掩模区域设置
    
    # 将图像转换为HSV颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 创建黄色颜色范围的掩模
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # 对黄色掩模应用道路掩模
    masked_yellow = create_mask(yellow_mask, mask_point)
    
    # 对黄色掩模进行边缘检测
    edges = cv2.Canny(masked_yellow, 50, 150)
    
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv2.contourArea(contour) > 50:  # 简单的面积过滤
            k, b = calculate_k_and_b(contour)
            angle = angle_calculate(k, b, frame)
            print("Detected angle:", angle)
            # 获取帧的宽度和高度
            height, width = frame.shape[:2]
            # 定义点的坐标
            center_point = (int(width / 2), int(height / 2))
            edge_point = (int(width / 4), int(k * width / 4))
            # 使用正确的坐标调用 cv2.line()
            edges = cv2.line(edges, center_point, edge_point, (255, 255, 0), 5)
            cv2.imshow('Edges', edges)
            # edges = cv2.line(edges,(frame.shape[1] / 2,frame.shape[0] / 2),(frame.shape[1] / 4,k * frame.shape[1] / 4),(0,255,0),5)  # 画出对应拟合斜率的线条
    cv2.imshow('Yellow Lanes', masked_yellow)

def on_trackbar(val):
    """ 滑动块回调函数 """
    pass

if __name__ == '__main__':

    # 创建滑动块窗口
    cv2.namedWindow('Yellow Range', cv2.WINDOW_NORMAL)
    
    cv2.createTrackbar('Hue Low', 'Yellow Range', 20, 179, on_trackbar)
    cv2.createTrackbar('Hue High', 'Yellow Range', 30, 179, on_trackbar)
    cv2.createTrackbar('Sat Low', 'Yellow Range', 100, 255, on_trackbar)
    cv2.createTrackbar('Sat High', 'Yellow Range', 255, 255, on_trackbar)
    cv2.createTrackbar('Val Low', 'Yellow Range', 100, 255, on_trackbar)
    cv2.createTrackbar('Val High', 'Yellow Range', 255, 255, on_trackbar)

    # 初始化滑动块位置
    cv2.setTrackbarPos('Hue Low', 'Yellow Range', 20)
    cv2.setTrackbarPos('Hue High', 'Yellow Range', 30)
    cv2.setTrackbarPos('Sat Low', 'Yellow Range', 100)
    cv2.setTrackbarPos('Sat High', 'Yellow Range', 255)
    cv2.setTrackbarPos('Val Low', 'Yellow Range', 100)
    cv2.setTrackbarPos('Val High', 'Yellow Range', 255)

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        hue_low = cv2.getTrackbarPos('Hue Low', 'Yellow Range')
        hue_high = cv2.getTrackbarPos('Hue High', 'Yellow Range')
        sat_low = cv2.getTrackbarPos('Sat Low', 'Yellow Range')
        sat_high = cv2.getTrackbarPos('Sat High', 'Yellow Range')
        val_low = cv2.getTrackbarPos('Val Low', 'Yellow Range')
        val_high = cv2.getTrackbarPos('Val High', 'Yellow Range')
        
        lower_yellow = np.array([hue_low, sat_low, val_low])
        upper_yellow = np.array([hue_high, sat_high, val_high])
        
        process_frame(frame, lower_yellow, upper_yellow)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()