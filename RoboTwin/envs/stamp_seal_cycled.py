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

class stamp_seal_cycled(Base_Task):
    """印章循环任务环境"""

    def setup_demo(self, **kwags):
        super()._init_task_env_(**kwags)

        self.info["info"] = {
            "{A}": "pad 1",
            "{B}": "pad 2",
            "{C}": "pad 3",
            "{D}": "pad 4",
            "{E}": "pad 5",
            "{F}": "pad 6",
            "{a}": "right",
        }

    def load_actors(self):

        # ---- 盖章顺序(可修改) ----
        self.pad_sequence = [1, 5, 3, 4, 2, 6]
        # ------------------------

        flushed_print("正在加载资产...")

        base_table_height = 0.74
        pad_z = base_table_height + 0.005

        # set pads
        """
        1 2 3 
        4 5 6 
        """
        self.config_pad = [
            [-0.04, -0.10, pad_z],
            [0.04, -0.10, pad_z],
            [0.12, -0.10, pad_z],
            [-0.04, -0.18, pad_z],
            [0.04, -0.18, pad_z],
            [0.12, -0.18, pad_z]
        ]
        pad_colors = [
            [1, 0, 0, 1],  # red
            [0, 0, 1, 1],  # blue
        ]
        pad_half_size = 0.035
        self.pads = []
        for i in range(6):
            pad = create_box(
                scene=self,
                pose=sapien.Pose(self.config_pad[i], [1, 0, 0, 0]),
                half_size=[pad_half_size, pad_half_size, 0.0025],
                color=pad_colors[i % len(pad_colors)],
                is_static=True,
                name=f"pad_{i+1}"
            )
            self.pads.append(pad)

        # set seal
        self.config_seal = {
            "pos": [0.2, -0.1, base_table_height + 0.1],
            "quat": [0.707, 0.707, 0, 0] # 指向上方
        }
        self.seal = create_actor(
            scene=self,
            pose=sapien.Pose(self.config_seal["pos"], self.config_seal["quat"]),
            modelname="100_seal",
            model_id=0,
            convex=True,
            is_static=False
        )

        # 初始化状态追踪
        self.full_sequence = self.pad_sequence * 2  # 完整序列 (2次循环)
        self.current_step_idx = 0  # 当前需要盖章的序列索引 (0 ~ 11)
        self.is_contacting = False # 当前接触状态 (防止重复计数)
        self.stamp_z_threshold = 0.02 # 接触判定的 Z 轴高度差阈值
        self.stamp_xy_threshold = 0.04 # 接触判定的 XY 平面距离阈值
        
        # 防止重复打印总体成功
        self.final_success_printed = False

        flushed_print("安全工作区初始化完成。")

    def take_dense_action(self, control_seq, save_freq=-1):
        """
        重写以在循环中包含 check_success，用于验证。
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
        flushed_print("开始环境演示...")
        arm_R = ArmTag("right")
        robot_ee_quat = [0.5, -0.5, 0.5, 0.5]

        pre_grasp_dis = 0.1

        # step 1: 抓取印章
        flushed_print("步骤 1: 抓取印章")
        
        self.move(self.grasp_actor(self.seal, arm_tag=arm_R, pre_grasp_dis=pre_grasp_dis))
        self.move(self.move_by_displacement(arm_R, z=0.08))

        # 循环盖章演示
        for i in range(2):
            for j, target_pad_idx in enumerate(self.pad_sequence):
                flushed_print(f"\n------------ 第 {i+1} 循环 第 {j+1} 次 (目标: 垫子 {target_pad_idx}) -------------")

                target_pad = self.pads[target_pad_idx - 1] # pad index is 0-5, sequence is 1-6
                target_pad_pose = target_pad.get_pose()

                # step 2: 移动到目标垫子上方
                flushed_print("步骤 2: 移动到目标垫子上方")
                pre_stamp_pose = sapien.Pose(
                    target_pad_pose.p[:3] + [0.0, 0.0, 0.23],
                    robot_ee_quat
                )
                self.move(self.move_to_pose(arm_R, pre_stamp_pose))

                # step 3: 盖章
                flushed_print("步骤 3: 盖章")
                
                # 下降接触
                self.move(self.move_to_pose(arm_R, sapien.Pose(
                    target_pad_pose.p[:3] + [0.0, 0.0, 0.185], # 稍微抬高一点防止压太死
                    robot_ee_quat
                )))

                # step 4: 抬起印章
                flushed_print("步骤 4: 抬起印章")
                self.move(self.move_by_displacement(arm_R, z=0.1))

            flushed_print(f"\n=== 已完成第 {i+1} 次完整盖章循环演示 ===")

        flushed_print("\n环境演示结束。")
        return self.info

    def check_success(self):
        # 防止初始化前检查
        if not hasattr(self, 'seal') or not hasattr(self, 'full_sequence'):
            return False

        # 如果已经完成全部任务
        if self.current_step_idx >= len(self.full_sequence):
            if not self.final_success_printed:
                flushed_print("\n✓ 任务成功：已完成两次完整盖章循环 (12次盖章)。")
                self.final_success_printed = True
            return True

        # 获取当前目标垫子
        # 序列存储的是 1-based index (1-6)，pads 列表是 0-based
        target_pad_num = self.full_sequence[self.current_step_idx]
        target_pad = self.pads[target_pad_num - 1]
        
        seal_pos = self.seal.get_pose().p
        pad_pos = target_pad.get_pose().p
        
        # 计算距离
        xy_dist = np.linalg.norm(seal_pos[:2] - pad_pos[:2])
        z_dist = abs(seal_pos[2] - pad_pos[2])
        
        # 判定是否"接触"：XY接近 且 Z高度差很小 (印章底部接触垫子)
        # 印章通常本身有一定高度，这里我们监控 seal actor 的原点与 pad 原点的距离
        # 假设印章是一个圆柱体，原点在几何中心或底部。从 play_once 的逻辑看，0.004 是判定阈值
        is_touching = (xy_dist < self.stamp_xy_threshold) and (z_dist < 0.05) 
        # 注意: z_dist < 0.05 是比较宽的，因为 seal 原点可能在中心。play_once 中是根据差值判定的
        # 更精确的方式是使用物理碰撞检测，或者根据实际模型尺寸调整 Z 阈值
        # 这里为了演示稳定性，使用位姿判定
        
        if is_touching:
            if not self.is_contacting:
                # 刚接触到 (Falling edge -> Rising edge)
                self.is_contacting = True
                flushed_print(f"✓ 检测到盖章动作：垫子 {target_pad_num} (进度: {self.current_step_idx + 1}/{len(self.full_sequence)})")
                self.current_step_idx += 1
        else:
            if self.is_contacting:
                # 刚离开 (Rising edge -> Falling edge)
                self.is_contacting = False # 重置，准备下一次接触

        # 总体检查
        if self.current_step_idx >= len(self.full_sequence):
             if not self.final_success_printed:
                flushed_print("\n✓ 任务成功：已完成两次完整盖章循环 (12次盖章)。")
                self.final_success_printed = True
             return True
             
        return False

if __name__ == "__main__":
    pass