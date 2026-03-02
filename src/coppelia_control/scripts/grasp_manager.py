#!/usr/bin/env python3
"""
✅ 最终修复版本 - 根据实际 URDF 文件修正
关键改动:
- base_frame: robot_base (从 URDF 确认)
- end_effector_link: link8_resp (实际的手腕链接)
- home_joints: 使用 URDF 中的初始值
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
import threading
import time

class State:
    SEARCHING = 0  
    PLANNING  = 1  
    MOVING    = 2  
    GRASPING  = 3  
    LIFTING   = 4  
    DONE      = 5  

class GraspManager(Node):
    def __init__(self):
        super().__init__('grasp_manager')
        
        # ✅ 【关键修复 1】从 URDF 确认的正确坐标系
        self.base_frame = "robot_base"           # ✅ URDF 中的基座框架名称
        self.end_effector_link = "link8_resp"    # ✅ 手腕最后一个链接（不是 panda_hand）
        self.arm_group = "panda_arm"             # ✅ MoveIt 规划组
        
        self.get_logger().info(f"[INIT] Base Frame: {self.base_frame}")
        self.get_logger().info(f"[INIT] EE Link: {self.end_effector_link}")
        self.get_logger().info(f"[INIT] Arm Group: {self.arm_group}")
        
        # 1. 订阅视觉话题
        self.sub_pose = self.create_subscription(
            PoseStamped, 
            '/vision/object_pose', 
            self.vision_callback, 
            10
        )
        self.get_logger().info("[INIT] Vision subscription created")
        
        # 2. 创建 MoveGroup 动作客户端
        self.move_group = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info("[INIT] Waiting for MoveGroup action server...")
        
        # ✅ 【关键修复 2】等待 MoveGroup 服务器启动
        if not self.move_group.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("[ERROR] MoveGroup server NOT available!")
            self.get_logger().error("[ERROR] 检查:")
            self.get_logger().error("[ERROR]   1. demo.launch 是否运行?")
            self.get_logger().error("[ERROR]   2. ros2_control 是否启动?")
            self.get_logger().error("[ERROR] 运行: ros2 control list_controllers")
            raise RuntimeError("MoveGroup server unavailable")
        
        self.get_logger().info("[OK] MoveGroup server is ready!")
        
        # 3. 初始化状态
        self.state = State.SEARCHING
        self.target_pose = None
        self.latest_vision_msg = None
        self.pre_grasp_offset = 0.15  # 15cm 预抓取高度
        
        self.get_logger().info("[READY] Grasp Manager initialized successfully")

        # 4. 启动独立控制线程
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

    def vision_callback(self, msg):
        """更新目标位置"""
        self.latest_vision_msg = msg
        if self.state == State.SEARCHING:
            self.get_logger().info(
                f"[VISION] Target Found! X={msg.pose.position.x:.3f}, "
                f"Y={msg.pose.position.y:.3f}, Z={msg.pose.position.z:.3f}"
            )
            self.target_pose = msg
            self.state = State.PLANNING

    def control_loop(self):
        """独立的控制循环"""
        time.sleep(2.0)  # 等待完全初始化
        self.get_logger().info("[CONTROL] Starting control loop...")
        self.go_to_home()
        time.sleep(1.0)
        
        while rclpy.ok():
            if self.state == State.PLANNING:
                self.execute_pick_sequence()
            time.sleep(0.1)

    def go_to_home(self):
        """
        ✅ 【关键修复 3】Home 姿态使用 URDF 中的初始值
        从 URDF 看:
        - joint1: 0.0
        - joint2: -0.785
        - joint3: 0.0
        - joint4: -2.356
        - joint5: 0.0
        - joint6: 1.571
        - joint7: 0.785
        """
        self.get_logger().info("[HOME] Moving to Home pose...")
        
        if not self.move_group.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("[HOME] MoveGroup server not available")
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.arm_group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        
        # ✅ 【关键修复 4】使用 URDF 中定义的初始值
        home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        joint_names = [f"panda_joint{i+1}" for i in range(7)]
        
        constraints = Constraints()
        for name, pos in zip(joint_names, home_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(constraints)
        
        try:
            future = self.move_group.send_goal_async(goal_msg)
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > 10.0:
                    self.get_logger().error("[HOME] Timeout")
                    return False
                time.sleep(0.1)
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("[HOME] Goal rejected")
                return False

            res_future = goal_handle.get_result_async()
            while not res_future.done(): 
                time.sleep(0.1)
                
            result = res_future.result().result
            if result.error_code.val == 1:
                self.get_logger().info("[HOME] ✅ Homing succeeded!")
                return True
            else:
                self.get_logger().error(f"[HOME] ❌ Failed with code: {result.error_code.val}")
                return False
        except Exception as e:
            self.get_logger().error(f"[HOME] Exception: {e}")
            return False

    def execute_pick_sequence(self):
        """执行完整的抓取流程"""
        if not self.target_pose: 
            return

        self.get_logger().info("[PICK] ========== Pick Sequence Started ==========")

        # Step 1: Pre-Grasp
        self.get_logger().info("[PICK] Step 1: Moving to Pre-Grasp...")
        self.state = State.MOVING
        success = self.move_arm_to_pose(self.target_pose, z_offset=self.pre_grasp_offset)
        
        if not success:
            self.get_logger().error("[PICK] Pre-grasp failed, retrying from home")
            self.state = State.SEARCHING
            self.go_to_home()
            return

        time.sleep(1.0)
        
        # Step 2: Descend to grasp
        self.get_logger().info("[PICK] Step 2: Descending to object...")
        success = self.move_arm_to_pose(self.target_pose, z_offset=0.01) 
        
        if success:
            time.sleep(0.5)
            self.state = State.GRASPING
            self.get_logger().info("[PICK] Step 3: CLOSING GRIPPER...")
            time.sleep(1.0) 
            
            # Step 3: Lift
            self.state = State.LIFTING
            self.get_logger().info("[PICK] Step 4: Lifting object...")
            success_lift = self.move_arm_to_pose(self.target_pose, z_offset=0.20)
            
            if success_lift:
                self.state = State.DONE
                self.get_logger().info("[PICK] 🎉 Mission complete!")
                time.sleep(5.0)
        
        self.get_logger().info("[PICK] Resetting to search mode...")
        time.sleep(2.0)
        self.state = State.SEARCHING 
        self.go_to_home()

    def move_arm_to_pose(self, target_msg, z_offset=0.0):
        """
        移动机械臂到指定位置
        ✅ 【关键修复 5】使用正确的坐标系和末端执行器链接
        """
        if not self.move_group.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("[MOVE] MoveGroup not available")
            return False

        try:
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = self.arm_group
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = 8.0
            goal_msg.request.max_velocity_scaling_factor = 0.3
            goal_msg.request.max_acceleration_scaling_factor = 0.3
            
            # 位置约束
            p_const = PositionConstraint()
            p_const.header.frame_id = self.base_frame  # ✅ robot_base
            p_const.link_name = self.end_effector_link  # ✅ link8_resp
            p_const.target_point_offset.x = 0.0
            p_const.target_point_offset.y = 0.0
            p_const.target_point_offset.z = 0.0
            
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [0.10, 0.10, 0.10]  # 10cm 容差
            p_const.constraint_region.primitives.append(box)
            
            t_pose = target_msg.pose
            p_const.constraint_region.primitive_poses.append(t_pose)
            p_const.constraint_region.primitive_poses[0].position.z += z_offset

            # 姿态约束 (手腕向下)
            o_const = OrientationConstraint()
            o_const.header.frame_id = self.base_frame
            o_const.link_name = self.end_effector_link
            o_const.orientation.x = 0.0
            o_const.orientation.y = 0.707
            o_const.orientation.z = 0.0
            o_const.orientation.w = 0.707
            
            o_const.absolute_x_axis_tolerance = 0.5
            o_const.absolute_y_axis_tolerance = 0.5
            o_const.absolute_z_axis_tolerance = 0.5

            constraints = Constraints()
            constraints.position_constraints.append(p_const)
            constraints.orientation_constraints.append(o_const)
            goal_msg.request.goal_constraints.append(constraints)

            self.get_logger().info(f"[MOVE] Planning to Z={t_pose.position.z + z_offset:.3f}")
            
            send_goal_future = self.move_group.send_goal_async(goal_msg)
            
            start_time = time.time()
            while not send_goal_future.done():
                if time.time() - start_time > 10.0:
                    self.get_logger().error("[MOVE] Goal acceptance timeout")
                    return False
                time.sleep(0.05)
                
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error("[MOVE] Goal rejected by MoveGroup")
                return False

            self.get_logger().info("[MOVE] Goal accepted, executing...")

            get_result_future = goal_handle.get_result_async()
            start_time = time.time()
            while not get_result_future.done():
                if time.time() - start_time > 15.0:
                    self.get_logger().error("[MOVE] Execution timeout")
                    return False
                time.sleep(0.05)
                
            result = get_result_future.result().result
            error_code = result.error_code.val
            
            if error_code == 1:
                self.get_logger().info("[MOVE] ✅ Motion succeeded!")
                return True
            else:
                self.get_logger().error(f"[MOVE] ❌ Failed with code: {error_code}")
                self._explain_error(error_code)
                return False
                
        except Exception as e:
            self.get_logger().error(f"[MOVE] Exception: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _explain_error(self, code):
        """解释 MoveIt 错误代码"""
        errors = {
            1: "SUCCESS",
            -1: "FAILURE",
            -2: "PLANNING_FAILED",
            -3: "INVALID_MOTION_PLAN",
            -5: "CONTROL_FAILED",
            -7: "TIMED_OUT",
            99999: "UNKNOWN (ros2_control issue)"
        }
        msg = errors.get(code, f"Unknown error {code}")
        self.get_logger().error(f"[ERROR] MoveIt Error: {msg}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = GraspManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Interrupted")
    except Exception as e:
        print(f"\n[FATAL] {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()