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
        
        # 参数配置：请根据你的 TF 树确认末端执行器的名字
        self.base_frame = "robot_base"
        self.ee_link = "FrankaGripper" # 或 panda_link8，取决于你的 MoveIt 配置

        self.get_logger().info("等待 MoveGroup 求解器...")
        self.move_group.wait_for_server(timeout_sec=10.0)
        self.get_logger().info("求解器就绪，准备盲抓！")

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
        self.get_logger().info(f">>> 规划笛卡尔目标: {desc} | Z={z:.3f}")
        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = vel
        goal.request.max_acceleration_scaling_factor = vel

        # 设置规划空间范围，防止报错
        wp = WorkspaceParameters()
        wp.header.frame_id = self.base_frame
        wp.min_corner.x, wp.min_corner.y, wp.min_corner.z = -2.0, -2.0, -2.0
        wp.max_corner.x, wp.max_corner.y, wp.max_corner.z = 2.0, 2.0, 2.0
        goal.request.workspace_parameters = wp

        # 构造位置约束 (Position)
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.ee_link
        
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.005] # 5毫米误差容忍度
        bv.primitives.append(sp)
        
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        bv.primitive_poses.append(p)
        pc.constraint_region = bv
        pc.weight = 1.0
        c.position_constraints.append(pc)

        # 构造姿态约束 (Orientation) - 夹爪垂直向下
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.ee_link
        oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = qx, qy, qz, qw
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        goal.request.goal_constraints.append(c)

        # 发送目标并等待执行
        f = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        gh = f.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f"❌ {desc} 规划无解 (IK Failed)")
            return False
            
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=20.0)
        ok = rf.result().result.error_code.val == 1
        if ok:
            self.get_logger().info(f"✅ {desc} 到达！")
            time.sleep(1.0) # 仿真缓冲时间
        return ok

    def run(self):
        # 夹爪朝下的标准四元数 (针对 Franka，如果姿态不对需要调整这个四元数)
        # 这里假设夹爪坐标系Z轴朝外，X轴向下，你需要根据实际 TF 调整
        qx, qy, qz, qw = 1.0, 0.0, 0.0, 0.0 

        # 刚才算出来的绝对物理坐标
        target_x = -0.625
        target_y = 0.000
        target_z = -0.050

        self.gripper(close=False)

        # 1. 移动到目标正上方 (预抓取，Z轴抬高15厘米)
        ok = self.move_to_pose(target_x, target_y, target_z + 0.15, qx, qy, qz, qw, "预抓取点", vel=0.2)
        if not ok: return

        # 2. 垂直下降接触方块
        ok = self.move_to_pose(target_x, target_y, target_z, qx, qy, qz, qw, "接触方块", vel=0.05)
        if not ok: return

        # 3. 闭合夹爪
        self.gripper(close=True)
        time.sleep(2.0)

        # 4. 垂直抬起
        self.move_to_pose(target_x, target_y, target_z + 0.20, qx, qy, qz, qw, "抓取抬升", vel=0.1)
        self.get_logger().info("🎉 盲抓流程执行完毕！")

def main():
    rclpy.init()
    node = AbsolutePick()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()