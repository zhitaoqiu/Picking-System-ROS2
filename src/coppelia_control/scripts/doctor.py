#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import time
import math

class PickOnce(Node):
    def __init__(self):
        super().__init__('pick_once')
        self.move_group = ActionClient(self, MoveGroup, 'move_action')
        self.target = None

        self.sub = self.create_subscription(
            PoseStamped, '/vision/object_pose',
            self.vision_cb, 10)

        self.gripper_pub = self.create_publisher(
            JointState, '/coppelia/gripper_cmd', 10)

        self.get_logger().info("等待 MoveGroup...")
        self.move_group.wait_for_server(timeout_sec=10.0)
        self.get_logger().info("就绪！")

    def vision_cb(self, msg):
        if self.target is None:
            self.target = msg
            self.get_logger().info(
                f"原始目标: X={msg.pose.position.x:.3f} "
                f"Y={msg.pose.position.y:.3f}")

    def move_joints(self, joints, desc, vel=0.05):
        self.get_logger().info(f">>> {desc}")
        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 8.0
        goal.request.max_velocity_scaling_factor = vel
        goal.request.max_acceleration_scaling_factor = vel

        c = Constraints()
        for i, pos in enumerate(joints):
            jc = JointConstraint()
            jc.joint_name = f"panda_joint{i+1}"
            jc.position = pos
            jc.tolerance_above = 0.15
            jc.tolerance_below = 0.15
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)

        f = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=15.0)
        gh = f.result()
        if not gh.accepted:
            self.get_logger().error(f"❌ {desc} 被拒绝")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=20.0)
        ok = rf.result().result.error_code.val == 1
        if ok:
            self.get_logger().info(f"✅ {desc} 成功，等待执行...")
            time.sleep(5.0)  # 给机械臂足够时间真正到位
        else:
            self.get_logger().error(f"❌ {desc} 失败")
        return ok

    def gripper(self, close=True):
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = ['openCloseJoint', 'centerJoint']
        cmd.position = [0.0, 0.0] if close else [0.04, 0.04]
        for _ in range(10):
            self.gripper_pub.publish(cmd)
            time.sleep(0.2)
        self.get_logger().info("夹爪已" + ("关闭" if close else "打开"))

    def run(self):
        # 等待视觉
        for _ in range(80):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.target:
                break

        if not self.target:
            self.get_logger().error("没有检测到目标！")
            return

        # 坐标取反修正相机方向
        x = -self.target.pose.position.x
        y = -self.target.pose.position.y
        j1 = math.atan2(y, x)
        self.get_logger().info(
            f"修正坐标: X={x:.3f} Y={y:.3f} "
            f"joint1={math.degrees(j1):.1f}度")

        # Step 0: 打开夹爪
        self.gripper(close=False)
        time.sleep(1.0)

        # Step 1: Home
        ok = self.move_joints(
            [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            "Home", vel=0.1)
        if not ok: return

        # Step 2: Pre-grasp
        ok = self.move_joints(
            [j1, -0.2, 0.0, -2.0, 0.0, 1.8, 0.785],
            "Pre-grasp", vel=0.08)
        if not ok: return

        # Step 3: 下降1
        ok = self.move_joints(
            [j1, 0.3, 0.0, -1.6, 0.0, 1.9, 0.785],
            "下降1", vel=0.06)
        if not ok: return

        # Step 4: 接触方块
        ok = self.move_joints(
            [j1, 0.55, 0.0, -1.3, 0.0, 1.85, 0.785],
            "接触", vel=0.05)
        if not ok:
            self.get_logger().warn("接触失败但继续夹取")

        # Step 5: 关夹爪
        self.get_logger().info("关闭夹爪！")
        self.gripper(close=True)
        time.sleep(3.0)

        # Step 6: 抬起
        ok = self.move_joints(
            [j1, -0.2, 0.0, -2.0, 0.0, 1.8, 0.785],
            "抬起", vel=0.06)
        if not ok: return

        # Step 7: 回Home
        self.move_joints(
            [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            "回Home", vel=0.1)

        self.get_logger().info("🎉 抓取完成！")

def main():
    rclpy.init()
    node = PickOnce()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()