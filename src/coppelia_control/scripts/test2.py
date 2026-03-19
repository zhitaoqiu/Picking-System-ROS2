#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, WorkspaceParameters
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import time

class AbsolutePick(Node):
    def __init__(self):
        super().__init__('absolute_pick')
        self.move_group = ActionClient(self, MoveGroup, 'move_action')
        self.gripper_pub = self.create_publisher(JointState, '/coppelia/gripper_cmd', 10)
        
        # 确保这里和你的 MoveIt 配置一致
        self.base_frame = "robot_base" 
        self.ee_link = "FrankaGripper" # 或 panda_link8

        self.get_logger().info("等待 MoveGroup 求解器...")
        self.move_group.wait_for_server(timeout_sec=10.0)
        self.get_logger().info("求解器就绪，准备突击盲抓！")

    def gripper(self, close=True):
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = ['openCloseJoint', 'centerJoint']
        cmd.position = [0.0, 0.0] if close else [0.04, 0.04]
        for _ in range(5):
            self.gripper_pub.publish(cmd)
            time.sleep(0.1)
        self.get_logger().info("夹爪 -> " + ("闭合" if close else "张开"))

    def move_to_pose(self, x, y, z, qx, qy, qz, qw, desc, vel=0.1):
        self.get_logger().info(f">>> 规划: {desc} | 坐标: [{x:.3f}, {y:.3f}, {z:.3f}]")
        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 8.0
        goal.request.max_velocity_scaling_factor = vel
        goal.request.max_acceleration_scaling_factor = vel

        wp = WorkspaceParameters()
        wp.header.frame_id = self.base_frame
        wp.min_corner.x, wp.min_corner.y, wp.min_corner.z = -2.0, -2.0, -2.0
        wp.max_corner.x, wp.max_corner.y, wp.max_corner.z = 2.0, 2.0, 2.0
        goal.request.workspace_parameters = wp

        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.ee_link
        
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01] # 允许 1 厘米的容差，防止求解器过于死板
        bv.primitives.append(sp)
        
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        bv.primitive_poses.append(p)
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.ee_link
        oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = qx, qy, qz, qw
        # 放宽一点姿态容差，确保一次规划成功
        oc.absolute_x_axis_tolerance = 0.15
        oc.absolute_y_axis_tolerance = 0.15
        oc.absolute_z_axis_tolerance = 0.15
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        goal.request.goal_constraints.append(c)

        f = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=15.0)
        gh = f.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f"❌ {desc} 规划无解 (IK Failed)")
            return False
            
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=20.0)
        ok = rf.result().result.error_code.val == 1
        if ok:
            self.get_logger().info(f"✅ {desc} 动作完成！")
            time.sleep(1.0)
        return ok

    def run(self):
        # 夹爪垂直朝下的标准四元数 (绕 X 轴旋转 180 度)
        qx, qy, qz, qw = 1.0, 0.0, 0.0, 0.0 

        # 结合物理世界换算出的绝对目标坐标！
        target_x = 0.325
        target_y = 0.025
        target_z = 0.060

        self.gripper(close=False)
        
        # 悬停在方块正上方 15cm 处
        ok = self.move_to_pose(target_x, target_y, target_z + 0.15, qx, qy, qz, qw, "预抓取点", vel=0.2)
        if not ok: return

        # 垂直下降接触方块
        ok = self.move_to_pose(target_x, target_y, target_z, qx, qy, qz, qw, "接触方块", vel=0.05)
        if not ok: return

        # 闭合夹爪
        self.gripper(close=True)
        time.sleep(2.0)

        # 垂直抬起完成抓取
        self.move_to_pose(target_x, target_y, target_z + 0.20, qx, qy, qz, qw, "抓取抬升", vel=0.1)
        self.get_logger().info("🎉 盲抓流程完美通关！")

def main():
    rclpy.init()
    node = AbsolutePick()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()