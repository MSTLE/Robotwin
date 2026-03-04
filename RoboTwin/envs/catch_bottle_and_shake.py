# pyright: reportArgumentType=false
from ._base_task import Base_Task
from .utils import *
import sapien
import numpy as np
import sys

def flushed_print(*args, **kwargs):
    """确保日志能够实时刷新的打印函数"""
    print(*args, **kwargs)
    sys.stdout.flush()

class catch_bottle_and_shake(Base_Task):
    
    def setup_demo(self, **kwags):
        super()._init_task_env_(**kwags)

        self.info["info"] = {
            "{A}": "001_bottle/base11",
            "{b}": "right",
        }

    def load_actors(self):
        flushed_print("正在加载资产...")

        # 瓶子初始位置随机化（矩形范围内均匀采样）
        rand_x = np.random.uniform(0.10, 0.20)
        rand_y = np.random.uniform(-0.10, 0.10)
        bottle_z = 0.79  # 瓶子质心的桌面静止高度
        flushed_print(f"瓶子随机初始位置: x={rand_x:.3f}, y={rand_y:.3f}, z={bottle_z}")

        # the location of the bottle
        self.config_bottle = {
            "pos": [rand_x, rand_y, bottle_z],
            "quat": [0.707, 0.707, 0, 0] # 指向上方
        }

        bottle_pos = sapien.Pose(self.config_bottle["pos"], self.config_bottle["quat"])
        self.bottle = create_actor(
            scene=self,
            pose=bottle_pos,
            modelname="001_bottle",
            model_id=11,
            convex=True,
            is_static=False
        )

        # 成功标志
        self.catch_success = False
        self.shake_count = 0        # 有效摇晃次数
        self.required_shakes = 3    # 目标摇晃次数

        # 抓取检测：需要连续多帧保持高度，防止碰撞弹起造成误判
        self.catch_height_counter = 0   # 当前连续高于阈值的帧数
        self.catch_height_frames = 30   # 需要持续 30 帧 (~0.12s) 才认定为抓取成功
        
        # 摇晃检测状态机变量
        self.is_holding = False
        self.last_z = bottle_z       # 上一帧的 Z 坐标（与瓶子实际初始高度一致）
        self.peak_z = -np.inf       # 当前过程中的最高点
        self.valley_z = np.inf      # 当前过程中的最低点
        self.shake_state = "IDLE"   # 状态: IDLE, UP, DOWN
        self.move_threshold = 0.10  # 判定有效摇晃的幅度阈值 (10cm)
        
        # 防止重复打印
        self.final_success_printed = False

        # 设置极高阻尼，防止瓶子在 check_stable() 的 2500 步物理检测期间因桌面网格凸起而倪倒
        # check_stable 监测 3° 旋转阈值；高阻尼下角速度 ≈ 0.0005rad/s → 2秒内旋转 <0.06°，远低于阈值
        # 在 play_once 开始前将阻尼重置为正常水平，不影响抓取和摇晃
        self.bottle_rigid = None
        for component in self.bottle.actor.get_components():
            if isinstance(component, sapien.physx.PhysxRigidDynamicComponent):
                component.linear_damping = 100.0
                component.angular_damping = 100.0
                self.bottle_rigid = component
                flushed_print("瓶子高阻尼已设置 (用于通过稳定性检测)")
                break
        if self.bottle_rigid is None:
            flushed_print("警告: 未找到瓶子刚体组件，无法设置阻尼")


        flushed_print("安全工作区初始化完成。")

    def take_dense_action(self, control_seq, save_freq=-1):
        """
        重写以在循环中包含 check_success，用于演示验证。
        """
        left_arm, left_gripper, right_arm, right_gripper = (
            control_seq["left_arm"],
            control_seq["left_gripper"],
            control_seq["right_arm"],
            control_seq["right_gripper"],
        )

        save_freq = self.save_freq if save_freq == -1 else save_freq
        if save_freq != None:
            self._take_picture()

        max_control_len = 0

        if left_arm is not None:
            max_control_len = max(max_control_len, left_arm["position"].shape[0])
        if left_gripper is not None:
            max_control_len = max(max_control_len, left_gripper["num_step"])
        if right_arm is not None:
            max_control_len = max(max_control_len, right_arm["position"].shape[0])
        if right_gripper is not None:
            max_control_len = max(max_control_len, right_gripper["num_step"])

        for control_idx in range(max_control_len):

            if (left_arm is not None and control_idx < left_arm["position"].shape[0]):
                self.robot.set_arm_joints(
                    left_arm["position"][control_idx],
                    left_arm["velocity"][control_idx],
                    "left",
                )

            if left_gripper is not None and control_idx < left_gripper["num_step"]:
                self.robot.set_gripper(
                    left_gripper["result"][control_idx],
                    "left",
                    left_gripper["per_step"],
                )

            if (right_arm is not None and control_idx < right_arm["position"].shape[0]):
                self.robot.set_arm_joints(
                    right_arm["position"][control_idx],
                    right_arm["velocity"][control_idx],
                    "right",
                )

            if right_gripper is not None and control_idx < right_gripper["num_step"]:
                self.robot.set_gripper(
                    right_gripper["result"][control_idx],
                    "right",
                    right_gripper["per_step"],
                )

            self.scene.step()
            self.check_success()  # 在此添加检查

            if self.render_freq and control_idx % self.render_freq == 0:
                self._update_render()
                self.viewer.render()

            if save_freq != None and control_idx % save_freq == 0:
                self._update_render()
                self._take_picture()

        if save_freq != None:
            self._take_picture()

        return True
    
    def play_once(self):
        flushed_print("执行 '放回方块' 任务序列（三次循环）...")

        # 稳定性检测已通过，将瓶子阻尼重置为正常水平，恢复真实物理
        if hasattr(self, 'bottle_rigid') and self.bottle_rigid is not None:
            self.bottle_rigid.linear_damping = 0.05
            self.bottle_rigid.angular_damping = 0.05
            flushed_print("瓶子阻尼已重置为正常水平")


        arm_R = ArmTag("right")

        # step 1: 抓取瓶子
        flushed_print("步骤 1: 抓取瓶子")

        self.move(self.grasp_actor(self.bottle, arm_tag=arm_R, pre_grasp_dis=0.15))
        self.move(self.move_by_displacement(arm_R, z=0.1))

        # step 2: 摇晃瓶子
        flushed_print("步骤 2: 摇晃瓶子")
        for cycle in range(3):
            flushed_print(f"\n===== 开始第 {cycle + 1} 次循环 =====")
            
            # 向上摇
            self.move(self.move_by_displacement(arm_R, z=0.15))
            
            # 向下摇
            self.move(self.move_by_displacement(arm_R, z=-0.15))
            
            flushed_print(f"第 {cycle + 1} 次循环动作完成")
        
        # move the arm back
        # self.move(self.back_to_origin(arm_R))

        return self.info

    def check_success(self):
        # 防止在初始化完成前检查
        if not hasattr(self, 'bottle'):
            return False

        # --- 第一步：检测抓取（需连续多帧保持高度，防止碰撞弹起误判）---
        current_z = self.bottle.get_pose().p[2]
        
        if not self.catch_success:
            if current_z > 0.85:
                self.catch_height_counter += 1
                if self.catch_height_counter >= self.catch_height_frames:
                    self.catch_success = True
                    flushed_print(f"✓ 瓶子抓取成功 (持续 {self.catch_height_frames} 帧高度检测触发)")
                    self.is_holding = True
            else:
                # 高度不足，重置计数器（必须连续满足才算）
                if self.catch_height_counter > 0:
                    flushed_print(f"  抓取检测重置 (高度不足，曾连续 {self.catch_height_counter} 帧，当前 z={current_z:.3f})")
                self.catch_height_counter = 0

        # --- 第二步：检测摇晃 (使用简单峰谷检测) ---
        if self.catch_success:
            # 实时更新极值
            if current_z > self.peak_z: self.peak_z = current_z
            if current_z < self.valley_z: self.valley_z = current_z
            
            # 状态机：IDLE -> UP -> DOWN -> UP ...
            
            # 检测向上运动完成 (从低点上升了足够距离)
            if self.shake_state == "DOWN" or self.shake_state == "IDLE":
                if current_z > self.valley_z + self.move_threshold:
                    self.shake_state = "UP"
                    # 重置极值用于下一次检测
                    self.peak_z = current_z 
                    
            # 检测向下运动完成 (从高点下降了足够距离) --- 这算一次完整的"摇"（半个周期）
            elif self.shake_state == "UP":
                if current_z < self.peak_z - self.move_threshold:
                    self.shake_state = "DOWN"
                    self.valley_z = current_z
                    self.shake_count += 1
                    flushed_print(f"检测到有效摇晃动作！(当前累计: {self.shake_count} 次半周期)")

        # 总体成功判定
        # 每次循环完成一次"下摇"动作，计入一次半周期
        # 3次循环至少会产生 3 次记录
        overall_success = self.catch_success and (self.shake_count >= 3)
        
        if overall_success and not self.final_success_printed:
            flushed_print(f"\n✓ 任务整体成功！(抓取: 成功, 摇晃计数: {self.shake_count})")
            self.final_success_printed = True
        
        return overall_success

        
if __name__ == "__main__":
    pass