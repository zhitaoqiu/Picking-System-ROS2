import numpy as np
from tf2_ros import Buffer, TransformListener
# 注意：虽然没直接用到，但必须导入它来注册 PoseStamped 的转换能力！
import tf2_geometry_msgs 
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from rclpy.duration import Duration

class CoordinateMapper:
    def __init__(self, node, fov_deg, target_z, target_frame='robot_base'):
        self.node = node
        self.target_z = target_z
        self.target_frame = target_frame
        
        self.w = 640
        self.h = 480
        self.fov = np.deg2rad(fov_deg)
        self.update_intrinsics(self.w, self.h)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def update_intrinsics(self, w, h):
        self.w = w
        self.h = h
        self.fx = (w / 2.0) / np.tan(self.fov / 2.0)
        self.fy = self.fx
        self.cx = w / 2.0
        self.cy = h / 2.0

    def pixel_to_world(self, u, v, src_frame, stamp):
        try:
            # 1. 构造相机坐标系下的 PoseStamped
            pose_cam = PoseStamped()
            pose_cam.header.frame_id = src_frame
            # 使用 Time() (0) 代表“最新时刻”，让 buffer 自己去匹配
            pose_cam.header.stamp = Time().to_msg()
            
            # 拆解赋值，确保不出错
            # 【恢复标准光学系映射】交给 TF 去处理旋转！
            # 图像左右(u) 对应 相机的 X
            pose_cam.pose.position.x = (u - self.cx) * 1.0 / self.fx
            
            # 图像上下(v) 对应 相机的 Y
            pose_cam.pose.position.y = (v - self.cy) * 1.0 / self.fy
            
            # 深度恒定为正前方的 1.0
            pose_cam.pose.position.z = 1.0 
            pose_cam.pose.orientation.w = 1.0

            # 2. 【核心修改】使用高阶 API 直接转换
            # 这行代码会自动查找 TF 并完成数学运算，绕过底层 bug
            pose_base = self.tf_buffer.transform(
                pose_cam, 
                self.target_frame, 
                timeout=Duration(seconds=1.0)
            )
            
            # 3. 射线计算 (后续逻辑不变)
            # 获取相机在基座下的位置 (射线起点)
            # 这里的 lookup_transform 只是为了拿相机原点，它是可靠的
            cam_tf = self.tf_buffer.lookup_transform(
                self.target_frame, 
                src_frame, 
                Time(), 
                timeout=Duration(seconds=1.0)
            )
            trans_vec = cam_tf.transform.translation
            cam_origin = np.array([trans_vec.x, trans_vec.y, trans_vec.z])
            
            # 射线终点
            p_end = np.array([pose_base.pose.position.x, 
                              pose_base.pose.position.y, 
                              pose_base.pose.position.z])
            
            ray_dir = p_end - cam_origin
            norm = np.linalg.norm(ray_dir)
            if norm < 1e-6: return None
            ray_dir = ray_dir / norm
            
            if abs(ray_dir[2]) < 1e-6: return None
            
            t = (self.target_z - cam_origin[2]) / ray_dir[2]
            
            if t < 0: return None
            
            final_pos = cam_origin + t * ray_dir
            
            # 4. 构造返回结果
            res_pose = PoseStamped()
            res_pose.header.frame_id = self.target_frame
            res_pose.header.stamp = stamp # 保持原时间戳
            
            res_pose.pose.position.x = final_pos[0]
            res_pose.pose.position.y = final_pos[1]
            res_pose.pose.position.z = self.target_z
            res_pose.pose.orientation.x = 1.0 
            res_pose.pose.orientation.w = 0.0
            
            return res_pose

        except Exception as e:
            # 打印错误，但这次应该不会再崩了
            import traceback
            self.node.get_logger().error(f"Vision Transform Error: {e}")
            self.node.get_logger().debug(traceback.format_exc())
            return None