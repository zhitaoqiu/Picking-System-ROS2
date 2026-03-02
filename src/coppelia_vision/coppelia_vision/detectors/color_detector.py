import cv2
import numpy as np

class ColorDetector:
    def __init__(self, cfg):
        self.conf = cfg.get('conf', 0.5)
        # 默认红色阈值 (CoppeliaSim Red)
        # H: 0-10 & 170-180
        self.lower1 = np.array([0, 100, 100])
        self.upper1 = np.array([10, 255, 255])
        self.lower2 = np.array([170, 100, 100])
        self.upper2 = np.array([180, 255, 255])

    def load_model(self):
        # 传统算法不需要加载权重，占位函数
        pass

    def infer(self, cv_img):
        """
        输入: BGR 图像
        输出: [{'label': 'red_cube', 'box': [x,y,w,h], 'center': (u,v)}, ...]
        """
        results = []
        
        # 1. 预处理
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        
        # 2. 颜色分割
        mask1 = cv2.inRange(hsv, self.lower1, self.upper1)
        mask2 = cv2.inRange(hsv, self.lower2, self.upper2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # 3. 形态学操作 (去噪)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 4. 轮廓查找
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 400: # 过滤太小的噪点
                x, y, w, h = cv2.boundingRect(cnt)
                
                # 计算重心
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = x + w//2, y + h//2
                
                results.append({
                    'label': 'red_cube',
                    'conf': 1.0,
                    'box': [x, y, w, h],
                    'center': (cX, cY),
                    'contour': cnt
                })
                
        return results