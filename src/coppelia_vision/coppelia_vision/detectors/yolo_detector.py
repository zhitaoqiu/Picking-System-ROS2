from .base_detector import BaseDetector
import cv2
# import torch  # 以后如果用 pytorch
# from ultralytics import YOLO # 如果用 ultralytics

class YoloDetector(BaseDetector):
    def __init__(self, config):
        super().__init__(config)
        self.model = None
        print(f"[YoloDetector] 初始化参数: {config}")

    def load_model(self):
        path = self.config.get('model_path', '')
        device = self.config.get('device', 'cpu')
        print(f"[YoloDetector] 正在从 {path} 加载模型到 {device}...")
        # TODO: 这里写真正的 YOLO 加载代码
        # self.model = YOLO(path) 
        # self.model.to(device)
        return True

    def infer(self, image):
        """
        返回格式示例:
        [
            {'label': 'cup', 'conf': 0.95, 'bbox': [x, y, w, h], 'center': (cx, cy)},
            ...
        ]
        """
        results = []
        
        # --- 模拟推理过程 (占位符) ---
        # 实际代码这里会调用 self.model(image)
        h, w, _ = image.shape
        center_x, center_y = w // 2, h // 2
        
        # 模拟识别到一个物体在画面中心
        results.append({
            'label': 'simulated_object',
            'conf': 0.99,
            'bbox': [center_x-50, center_y-50, 100, 100],
            'center': (center_x, center_y)
        })
        # ---------------------------
        
        return results