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
        # 加入整体位移随机化
        pad_offset_x = np.random.uniform(-0.02, 0.02)
        pad_offset_y = np.random.uniform(-0.02, 0.02)

        self.config_pad = [
            [-0.04 + pad_offset_x, -0.10 + pad_offset_y, pad_z],
            [0.04 + pad_offset_x, -0.10 + pad_offset_y, pad_z],
            [0.12 + pad_offset_x, -0.10 + pad_offset_y, pad_z],
            [-0.04 + pad_offset_x, -0.18 + pad_offset_y, pad_z],
            [0.04 + pad_offset_x, -0.18 + pad_offset_y, pad_z],
            [0.12 + pad_offset_x, -0.18 + pad_offset_y, pad_z]
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
            # "pos": [0.2, -0.1, base_table_height + 0.1],
            "pos": [np.random.uniform(0.2, 0.3), np.random.uniform(-0.3, -0.1), base_table_height + 0.1],
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
        self.stage_id = 0  # 当前需要盖章的序列索引 (0 ~ 11)
        self.stamp_flag = False # 目前是否正在压着垫子
        self.last_stamped_pad = -1 # 记录上一个合法压下的垫子索引，防拖拽
        self.stamp_z_threshold = 0.02 # 接触判定的 Z 轴高度差阈值
        self.stamp_xy_threshold = 0.04 # 接触判定的 XY 平面距离阈值
        self.fail_flag = False # 记录是否因为盖错顺序/拖拽等投机行为而任务失败
        self.max_reward = 0.0
        
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

        # put the seal back to the initial position and move the grasp back
        self.move(self.move_to_pose(arm_R, sapien.Pose(
                    np.array(self.config_seal["pos"]) + [0.0, 0.0, 0.245], 
                    robot_ee_quat
                )))
        self.move(self.move_by_displacement(arm_R, z=-0.16))
        self.move(self.open_gripper(arm_tag=arm_R))
        self.move(self.back_to_origin(arm_tag=arm_R))

        flushed_print("\n环境演示结束。")
        return self.info

    def check_stamp_pressed(self, pad_id, z_threshold=0.03, xy_threshold=0.03):
        seal_pos = self.seal.get_pose().p
        pad_pos = self.pads[pad_id].get_pose().p
        xy_dist = np.linalg.norm(seal_pos[:2] - pad_pos[:2])
        z_dist = abs(seal_pos[2] - pad_pos[2])
        if (xy_dist < xy_threshold) and (z_dist < z_threshold):
            return True
        return False

    def update_stamp_reset(self, safe_z_threshold=0.08):
        seal_pos = self.seal.get_pose().p
        is_lifted = True
        for pad in self.pads:
            pad_pos = pad.get_pose().p
            if abs(seal_pos[2] - pad_pos[2]) < safe_z_threshold:
                is_lifted = False
                break
        if is_lifted:
            self.stamp_flag = False

    def check_success(self):
        # 0. 生命周期保护：确保在 load_actors 执行后才运行逻辑
        if not hasattr(self, 'stage_id') or not hasattr(self, 'full_sequence') or not hasattr(self, 'seal'):
            return False

        # 1. 永久失败检查
        if getattr(self, 'fail_flag', False):
            return False
            
        # ---------- 修正后的逻辑顺序 ----------
        # 1. 更新抬起状态 (磁滞区间机制)
        self.update_stamp_reset()

        # 2. 检查：是否已经在最后阶段，且发生“画蛇添足”（越界多盖）
        if self.stage_id == len(self.full_sequence):
            for i in range(6):
                # 如果检测到此时又压在了任何一个垫子上
                if self.check_stamp_pressed(i) and not self.stamp_flag:
                    flushed_print(f"❌ 失败：画蛇添足！要求盖章 {len(self.full_sequence)} 次，实际却检测到了多余的盖章。")
                    self.fail_flag = True
                    return False

            # 如果没有乱压垫子，此时检查是否印章已经放回原位
            seal_pos = self.seal.get_pose().p
            init_pos = np.array(self.config_seal["pos"])
            seal_xy_dist = np.linalg.norm(seal_pos[:2] - init_pos[:2])
            seal_z = seal_pos[2]
            
            is_seal_returned = (seal_xy_dist < 0.05) and (seal_z < 0.76)
            
            if is_seal_returned:
                if hasattr(self, 'max_reward'):
                    self.max_reward = max(self.max_reward, 1.0)
                if not getattr(self, 'final_success_printed', False):
                    flushed_print("\n✓ 任务成功：已完成两次完整盖章循环 (12次有效规则盖章)。")
                    self.final_success_printed = True
                return True
            else:
                return False # 还在返回途中或未进行下一步
        
        elif self.stage_id > len(self.full_sequence):
            self.fail_flag = True
            return False

        # 计算当前步对应的目标
        expected_pad_num = self.full_sequence[self.stage_id]
        expected_pad_id = expected_pad_num - 1 

        # 5. 正确动作判定
        if self.check_stamp_pressed(expected_pad_id) and not self.stamp_flag:
            flushed_print(f"✓ 正确：检测到盖章动作 垫子 {expected_pad_num} (进度: {self.stage_id + 1}/{len(self.full_sequence)})")
            self.stamp_flag = True 
            self.last_stamped_pad = expected_pad_id # 记录锚点，防拖拽
            self.stage_id += 1     
            return False           

        # 6. 防乱盖与防贴地拖拽监控
        for i in range(6):
            if self.check_stamp_pressed(i):
                if self.stamp_flag:
                    # 如果没抬起来就滑到了别的垫子上，视为拖拽作弊
                    if i != getattr(self, 'last_stamped_pad', -1):
                        flushed_print(f"❌ 失败：检测到拖拽或未充分抬起动作！违规触碰垫子 {i + 1}")
                        self.fail_flag = True
                        return False
                else:
                    # 如果已经抬起，但重新压下的不是目标垫子，视为盖错顺序
                    # (由于正确的已经在第4步拦截了，这里捕获的必定是错的)
                    flushed_print(f"❌ 失败：按错了垫子顺序！实际触碰 {i + 1}")
                    self.fail_flag = True
                    return False

        return False

if __name__ == "__main__":
    pass