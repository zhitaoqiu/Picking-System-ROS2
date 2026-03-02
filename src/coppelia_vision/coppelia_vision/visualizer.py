import cv2
import numpy as np

class Visualizer:
    def __init__(self):
        pass

    def draw_results(self, img, results):
        vis_img = img.copy()
        
        for obj in results:
            # 画框
            x, y, w, h = obj['box']
            cv2.rectangle(vis_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # 画中心点
            cX, cY = obj['center']
            cv2.circle(vis_img, (cX, cY), 5, (0, 0, 255), -1)
            
            # 画文字 (如果有 3D 坐标则显示坐标，否则显示标签)
            text = obj['label']
            if 'pose' in obj:
                p = obj['pose'].pose.position
                text = f"[{p.x:.2f}, {p.y:.2f}, {p.z:.2f}]"
            
            cv2.putText(vis_img, text, (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        return vis_img