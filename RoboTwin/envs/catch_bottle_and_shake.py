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
            "task": "catch_bottle_and_shake",
            "{A}": "bottle",
            "{B}": "table",
            "{a}": "left arm",
            "{b}": "right arm"
        }

    def load_actors(self):
        flushed_print("正在加载资产...")

        # the location of the bottle
        self.config_bottle = {
            "pos": [0.15, 0.0, 0.79],
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
        
        # 摇晃检测状态机变量
        self.is_holding = False
        self.last_z = 0.79          # 上一帧的 Z 坐标
        self.peak_z = -np.inf       # 当前过程中的最高点
        self.valley_z = np.inf      # 当前过程中的最低点
        self.shake_state = "IDLE"   # 状态: IDLE, UP, DOWN
        self.move_threshold = 0.10  # 判定有效摇晃的幅度阈值 (10cm)
        
        # 防止重复打印
        self.final_success_printed = False

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

        # --- 第一步：检测抓取 ---
        current_z = self.bottle.get_pose().p[2]
        
        # 判定抓取成功：瓶子高度 > 0.85 (被提离桌面约 6cm)
        if not self.catch_success and current_z > 0.85:
            self.catch_success = True
            flushed_print("✓ 瓶子抓取成功 (高度检测触发)")
            self.is_holding = True # 开始摇晃检测

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