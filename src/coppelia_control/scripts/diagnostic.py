#!/usr/bin/env python3
"""
ROS2 + MoveIt 诊断脚本
检查: 坐标系树、控制器状态、节点连接
"""
import rclpy
from rclpy.node import Node
import subprocess
import time

class DiagnosticNode(Node):
    def __init__(self):
        super().__init__('diagnostic_node')
        
    def run_diagnostics(self):
        """运行完整诊断"""
        
        print("\n" + "="*60)
        print("🔍 ROS2 + MoveIt 诊断报告")
        print("="*60)
        
        # 1. 检查 ROS 节点
        print("\n[1️⃣] 活跃的 ROS 节点:")
        print("-" * 60)
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                    capture_output=True, text=True, timeout=5)
            nodes = result.stdout.strip().split('\n')
            important_nodes = ['move_group', 'rviz', 'ros2_control', 'joint_state_broadcaster']
            
            for node in nodes:
                if node:
                    status = "✅" if any(imp in node for imp in important_nodes) else "  "
                    print(f"{status} {node}")
            
            if 'move_group' not in result.stdout:
                print("⚠️  WARNING: move_group node not found! Is demo.launch running?")
        except Exception as e:
            print(f"❌ Error: {e}")
        
        # 2. 检查 ros2_control 控制器
        print("\n[2️⃣] ros2_control 控制器状态:")
        print("-" * 60)
        try:
            result = subprocess.run(['ros2', 'control', 'list_controllers'], 
                                    capture_output=True, text=True, timeout=5)
            print(result.stdout)
            
            if not result.stdout.strip() or 'active' not in result.stdout.lower():
                print("❌ ERROR: No active controllers found!")
                print("   Action: Launch ros2_control with demo.launch")
        except Exception as e:
            print(f"❌ Error: {e}")
        
        # 3. 检查关键话题
        print("\n[3️⃣] 关键 ROS 话题:")
        print("-" * 60)
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                    capture_output=True, text=True, timeout=5)
            topics = result.stdout.strip().split('\n')
            
            critical_topics = {
                '/joint_states': '关节状态',
                '/move_group/goal': '规划目标',
                '/rviz_moveit_motion_planning_display/robot_state': '机械臂状态',
                '/camera/image_raw': '摄像头图像',
                '/vision/object_pose': '物体位置'
            }
            
            for topic, desc in critical_topics.items():
                exists = any(topic in t for t in topics)
                status = "✅" if exists else "⚠️"
                print(f"{status} {topic:45} ({desc})")
            
        except Exception as e:
            print(f"❌ Error: {e}")
        
        # 4. 检查坐标系树
        print("\n[4️⃣] TF 坐标系树 (关键帧):")
        print("-" * 60)
        try:
            result = subprocess.run(['ros2', 'tf2', 'list_frames'], 
                                    capture_output=True, text=True, timeout=5)
            frames = result.stdout.strip().split('\n')
            
            # 寻找关键帧
            key_frames = ['panda_link0', 'world', 'base_link', 'panda_hand', 
                         'Franka', 'base', 'robot_base', 'panda_joint']
            
            found_frames = [f for f in frames if any(k in f for k in key_frames)]
            
            print("找到的关键帧:")
            for frame in found_frames[:15]:  # 只显示前15个
                print(f"  - {frame}")
            
            if not found_frames:
                print("⚠️  WARNING: No key frames found!")
                print("所有帧:")
                for frame in frames[:20]:
                    print(f"  - {frame}")
                    
        except Exception as e:
            print(f"⚠️  Warning (可能 TF 还未完全初始化): {e}")
        
        # 5. 检查 MoveGroup 连接
        print("\n[5️⃣] MoveGroup 服务检查:")
        print("-" * 60)
        try:
            result = subprocess.run(['ros2', 'service', 'list'], 
                                    capture_output=True, text=True, timeout=5)
            
            services = result.stdout.strip().split('\n')
            moveit_services = [s for s in services if 'move_group' in s or 'planning' in s]
            
            if moveit_services:
                print("✅ MoveGroup 服务找到:")
                for svc in moveit_services[:10]:
                    print(f"  - {svc}")
            else:
                print("❌ ERROR: No MoveGroup services found!")
                
        except Exception as e:
            print(f"❌ Error: {e}")
        
        # 6. 检查 CoppeliaSim 连接
        print("\n[6️⃣] CoppeliaSim 连接状态:")
        print("-" * 60)
        try:
            result = subprocess.run(['ros2', 'topic', 'echo', '/joint_states', '-n', '1'],
                                    capture_output=True, text=True, timeout=3)
            if result.returncode == 0:
                print("✅ CoppeliaSim 正在发送关节状态")
                # 解析关节数
                if 'position' in result.stdout:
                    print("   (正在接收数据)")
            else:
                print("⚠️  WARNING: 无法接收 /joint_states")
                print("   Action: 确认 CoppeliaSim 和 ROS2 接口已启动")
        except Exception as e:
            print(f"⚠️  Warning: {e}")
        
        # 7. 检查参数
        print("\n[7️⃣] MoveIt 关键参数:")
        print("-" * 60)
        try:
            result = subprocess.run(['ros2', 'param', 'list'], 
                                    capture_output=True, text=True, timeout=5)
            
            params = result.stdout.strip().split('\n')
            key_params = [p for p in params if 'panda' in p.lower() or 'robot' in p.lower()]
            
            if key_params:
                print("✅ 机械臂配置参数:")
                for param in key_params[:10]:
                    print(f"  - {param}")
            else:
                print("⚠️  No robot parameters found")
                
        except Exception as e:
            print(f"⚠️  Warning: {e}")
        
        # 8. 建议
        print("\n[⚠️] 故障排查建议:")
        print("-" * 60)
        print("""
如果看到上面的错误，按这个顺序检查:

1️⃣  启动顺序错误?
   - 先启动 CoppeliaSim 和 ros2_control
   - 再启动 demo.launch
   - 最后启动你的抓取脚本

2️⃣  坐标系错误?
   - 检查 [4️⃣] 中的关键帧
   - 修改脚本中的 self.base_frame
   - 常见值: "panda_link0", "world", "base_link", "Franka"

3️⃣  控制器未启动?
   - 检查 [2️⃣] 中的控制器列表
   - 如果为空，ros2_control 未正确启动
   - 查看 demo.launch 的配置

4️⃣  MoveGroup 无响应?
   - 检查 [5️⃣] 的服务列表
   - 查看 move_group 的完整日志:
     ros2 launch panda_moveit_config demo.launch --log-level debug

5️⃣  CoppeliaSim 未连接?
   - 检查 [6️⃣] 的状态
   - 确保 CoppeliaSim 在运行
   - 检查 ROS2-CoppeliaSim 插件是否正确安装
        """)
        
        print("\n" + "="*60)
        print("诊断完成")
        print("="*60 + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticNode()
    node.run_diagnostics()
    rclpy.shutdown()

if __name__ == '__main__':
    main()