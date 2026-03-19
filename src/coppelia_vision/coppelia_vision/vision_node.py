#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

# 导入功能模块
from coppelia_vision.coordinate_mapper import CoordinateMapper
from coppelia_vision.visualizer import Visualizer
from coppelia_vision.detectors.color_detector import ColorDetector

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # 1. 参数配置
        self.declare_parameters(namespace='', parameters=[
            ('sub_image_topic', '/global_camera/image'),
            ('pub_result_topic', '/vision/object_pose'),
            ('base_frame', 'robot_base'), 
            ('camera_frame', 'global_camera_link'),
            ('model_type', 'color'),
            ('model_path', ''),
            ('device', 'cpu'),
            ('conf_threshold', 0.5),
            ('enable_visualization', True),
            ('fov_degrees', 60.0),
            ('target_z_height', 0.02)
        ])
        
        # 2. 初始化核心模块
        base_frame = self.get_parameter('base_frame').value
        fov = self.get_parameter('fov_degrees').value
        z_h = self.get_parameter('target_z_height').value
        
        self.bridge = CvBridge()
        self.mapper = CoordinateMapper(self, fov, z_h, target_frame=base_frame) 
        self.vis = Visualizer()
        self.init_detector()

        # 3. 通信建立 (恢复标准 Reliable 模式)
        topic_name = self.get_parameter('sub_image_topic').value
        self.get_logger().info(f"Vision Node Init | Listening to: {topic_name}")

        self.sub = self.create_subscription(
            Image, 
            topic_name, 
            self.img_callback, 
            10  # 标准队列长度，默认就是 Reliable
        )
        self.pub_res = self.create_publisher(PoseStamped, self.get_parameter('pub_result_topic').value, 10)

    def init_detector(self):
        m_type = self.get_parameter('model_type').value
        cfg = {'conf': self.get_parameter('conf_threshold').value}
        self.detector = ColorDetector(cfg)
        if hasattr(self.detector, 'load_model'):
            self.detector.load_model()

    def img_callback(self, msg):
        try:
            # 1. 安全解码 (防止 buffer 长度不一致报错)
            expected_size = msg.height * msg.width * 3
            if len(msg.data) != expected_size:
                return 

            # 使用 Numpy 转换 (比 cv_bridge 更快更稳)
            np_img = np.array(msg.data, dtype=np.uint8)
            cv_img = np_img.reshape((msg.height, msg.width, 3))
            
            if msg.encoding == 'rgb8':
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
            
            # 2. 视觉管线
            self.mapper.update_intrinsics(msg.width, msg.height)
            results = self.detector.infer(cv_img)
            
            for obj in results:
                u, v = obj['center']
                # 优先用消息帧，参数帧作备选
                src_frame = msg.header.frame_id if msg.header.frame_id else self.get_parameter('camera_frame').value
                
                # 坐标映射
                pose_3d = self.mapper.pixel_to_world(u, v, src_frame, msg.header.stamp)
                
                if pose_3d:
                    obj['pose'] = pose_3d
                    self.pub_res.publish(pose_3d)
                    
                    # 仅打印必要信息
                    pos = pose_3d.pose.position
                    self.get_logger().info(f"📍 Cube: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]")
            
            # 3. 可视化
            if self.get_parameter('enable_visualization').value:
                debug_img = self.vis.draw_results(cv_img, results)
                cv2.imshow("Robot Eye", debug_img)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Processing Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()