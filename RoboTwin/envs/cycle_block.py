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

class cycle_block(Base_Task):
    def setup_demo(self, **kwags):
        super()._init_task_env_(**kwags)

        self.info["info"] = {
            "{A}": "pad0",
            "{B}": "pad1",
            "{C}": "pad2",
            "{D}": "red block",
            "{E}": "green block",
            "{a}": "left",
            "{b}": "right",
        }

    def load_actors(self):
        flushed_print("正在加载资产...")

        base_table_height = 0.74

        # step1: set 3 pad in the table
        """
                  pad 0
            pad 1       pad 2
        """
        pad_z = base_table_height + 0.001

        self.pad_poses = [
            [np.random.uniform(-0.03, 0.03), np.random.uniform(-0.07, -0.03), pad_z],    # pad 0
            [np.random.uniform(-0.07, -0.03), np.random.uniform(-0.17, -0.13), pad_z],    # pad 1
            [np.random.uniform(0.03, 0.07), np.random.uniform(-0.17, -0.13), pad_z]      # pad 2
        ]
        self.pads = []

        from .utils.create_actor import create_visual_box
        for i, pos in enumerate(self.pad_poses):
            flushed_print(f"Pad {i} 位置: {pos}")
            pad = create_visual_box(
                scene=self.scene,
                pose=sapien.Pose(self.pad_poses[i]),
                half_size=(0.04, 0.04, 0.001),
                color=[45/255, 173/255, 232/255],  # blue
                name=f"pad{i}",
            )
            self.pads.append(pad)

        # step2: 在 pad1 和 pad2 上放置 2 个方块
        block_size = 0.02
        block_z = base_table_height + block_size
        block_positions = [
            ([0.0, -0.050, block_z], "block0"),  # no used
            ([self.pad_poses[1][0], self.pad_poses[1][1], block_z], "block1"),
            ([self.pad_poses[2][0], self.pad_poses[2][1], block_z], "block2")
        ]

        block_index = 1  # block1 in pad1, block2 in pad2, pad0 is empty

        self.selected_indices = [block_index, (block_index + 1) % 3]  # [1, 2]

        self.blocks = {}

        colors = [
            [232/255, 45/255, 73/255],  # red
            [45/255, 232/255, 73/255]   # green
        ]

        for i, idx in enumerate(self.selected_indices):
            pos, name = block_positions[idx]
            block = create_box(
                scene=self,
                pose=sapien.Pose(pos),
                half_size=(block_size, block_size, block_size),
                color=colors[i],
                name=name,
                is_static=False
            )
            block.set_mass(0.01)
            self.blocks[name] = [block, idx]
            flushed_print(f"已生成 {name} 位置: {pos} 颜色: {colors[i]}")

        self.empty_pad = (block_index + 2) % 3  # = 0

        # ---- 状态机初始化（全部在 load_actors 中完成，不在 check_success 懒加载）----

        # 目标步数
        self.target_move_count = 12  # 2轮 × 3次/轮 × 2个方块

        # 预计算完整的 12 步移动序列
        self.move_sequence = self._compute_move_sequence()

        self.stage_id = 0       # 当前已完成的有效移动次数 (0 ~ 12)
        self.fail_flag = False  # 永久失败标志
        self.max_reward = 0.0

        # 初始 pad 状态
        # block1 初始在 pad1，block2 初始在 pad2
        self.block_on_pad = {
            f"block{self.selected_indices[0]}": self.selected_indices[0],  # block1 in pad1
            f"block{self.selected_indices[1]}": self.selected_indices[1],  # block2 in pad2
        }

        # 磁滞标志：方块是否已被"抬起"（类比 stamp_seal_cycled 的 stamp_flag）
        # False = 方块在 block_on_pad 记录的位置未被移动
        # True  = 方块已被抬起，等待落地确认
        self.block_lifted = {name: False for name in self.block_on_pad}

        # 判定参数
        self.pad_xy_threshold = 0.02     # XY 距离阈值 (m)
        self.on_table_z_max = block_z + 0.02  # Z 阈值：高于此为"在空中"

        self.final_success_printed = False

        flushed_print("安全工作区初始化完成。")

    def _compute_move_sequence(self):
        """
        预计算完整的 12 步移动序列。
        模拟 play_once 的演示逻辑，返回 [(block_name, from_pad, to_pad), ...]。
        """
        sequence = []
        # 模拟初始状态
        blk_pad = {
            f"block{self.selected_indices[0]}": self.selected_indices[0],
            f"block{self.selected_indices[1]}": self.selected_indices[1],
        }
        empty = self.empty_pad

        for _ in range(2):   # 2 轮
            for _ in range(3):  # 每轮 3 次（与 play_once 的 range(3) 对应）
                # play_once 中先移 selected_indices[1] (block2)
                b1 = f"block{self.selected_indices[1]}"
                from1, to1 = blk_pad[b1], empty
                sequence.append((b1, from1, to1))
                blk_pad[b1] = to1
                empty = from1

                # 再移 selected_indices[0] (block1)
                b2 = f"block{self.selected_indices[0]}"
                from2, to2 = blk_pad[b2], empty
                sequence.append((b2, from2, to2))
                blk_pad[b2] = to2
                empty = from2

        return sequence

    def take_dense_action(self, control_seq, save_freq=-1):
        """重写以在循环中包含 check_success，用于验证。"""
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
        arm_L = ArmTag("left")
        arm_R = ArmTag("right")
        change_arm = False

        # 0 -> 1: arm_L
        # 1 -> 2: arm_R
        # 2 -> 0: arm_R
        # 演示逻辑：每步根据 pos_id 选择手臂，循环 2 轮 × 3 次
        self.arm_before = None
        arm_use = None
        for i in range(2):
            self.success_catch_times = 0
            self.success_place_times = 0

            pre_grasp_dis = 0.1
            grasp_dis = 0.03
            MOVE_UP_AFTER_GRASP = 0.05
            robot_quat = [0.5, -0.5, 0.5, 0.5]
            place_down_distance = 0.081

            for j in range(3):
                # 抓第一个方块（selected_indices[1]）
                first_block, pos_id = self.blocks[f"block{self.selected_indices[1]}"]
                self.arm_before = arm_use
                arm_use = arm_L if pos_id == 0 else arm_R
                if self.arm_before is not None and self.arm_before != arm_use:
                    change_arm = True

                self.move(self.grasp_actor(first_block, arm_tag=arm_use, pre_grasp_dis=pre_grasp_dis, grasp_dis=grasp_dis),
                          self.back_to_origin(self.arm_before) if change_arm else None)
                change_arm = False
                self.move(self.move_by_displacement(arm_use, z=MOVE_UP_AFTER_GRASP))

                # 更新内部追踪 (用于驱动演示逻辑，不影响评分)
                self.blocks[f"block{self.selected_indices[1]}"][1] += 1
                self.blocks[f"block{self.selected_indices[1]}"][1] %= 3

                # place first block
                target_pos_pre = self.pad_poses[self.empty_pad].copy()
                target_pos_pre[2] += 0.25
                self.move(self.move_to_pose(arm_use, sapien.Pose(target_pos_pre, robot_quat)))
                self.move(self.move_by_displacement(arm_use, z=-place_down_distance))
                self.move(self.open_gripper(arm_use))
                self.move(self.move_by_displacement(arm_use, z=MOVE_UP_AFTER_GRASP))

                # 更新空Pad索引
                self.empty_pad = (self.empty_pad + 2) % 3

                # 抓第二个方块（selected_indices[0]）
                second_block, pos_id = self.blocks[f"block{self.selected_indices[0]}"]
                self.arm_before = arm_use
                arm_use = arm_L if pos_id == 0 else arm_R
                if self.arm_before is not None and self.arm_before != arm_use:
                    change_arm = True

                self.move(self.grasp_actor(second_block, arm_tag=arm_use, pre_grasp_dis=pre_grasp_dis, grasp_dis=grasp_dis),
                          self.back_to_origin(self.arm_before) if change_arm else None)
                change_arm = False
                self.move(self.move_by_displacement(arm_use, z=MOVE_UP_AFTER_GRASP))

                # 更新内部追踪
                self.blocks[f"block{self.selected_indices[0]}"][1] += 1
                self.blocks[f"block{self.selected_indices[0]}"][1] %= 3

                # 放置第二个方块到空 pad
                target_pos_pre = self.pad_poses[self.empty_pad].copy()
                target_pos_pre[2] += 0.25
                self.move(self.move_to_pose(arm_use, sapien.Pose(target_pos_pre, robot_quat)))
                self.move(self.move_by_displacement(arm_use, z=-place_down_distance))
                self.move(self.open_gripper(arm_use))
                self.move(self.move_by_displacement(arm_use, z=MOVE_UP_AFTER_GRASP))

                # 更新空Pad索引
                self.empty_pad = (self.empty_pad + 2) % 3

            flushed_print(f"=== 第 {i+1} 轮循环演示完成 ===")

        self.move(self.back_to_origin(arm_use))
        flushed_print("演示完成，已返回初始位置")

        return self.info

    # ------------------------------------------------------------------ #
    #  辅助函数（类比 stamp_seal_cycled 的 check_stamp_pressed / update）  #
    # ------------------------------------------------------------------ #

    def check_is_on_pad(self, block_name, pad_id):
        """
        检查 block_name 是否低空落在 pad_id 上。
        XY 距离 < pad_xy_threshold 且 Z ≤ on_table_z_max。
        （类比 stamp_seal_cycled.check_stamp_pressed）
        """
        block_obj = self.blocks[block_name][0]
        block_pos = block_obj.get_pose().p
        pad_pos = self.pad_poses[pad_id]
        xy_dist = np.linalg.norm(block_pos[:2] - np.array(pad_pos[:2]))
        return xy_dist < self.pad_xy_threshold and block_pos[2] <= self.on_table_z_max

    def update_block_lifted(self, block_name):
        """
        如果方块 Z 高于阈值，标记为已抬起（block_lifted = True）。
        仅单向置位；复位由状态机在确认落地后完成。
        （类比 stamp_seal_cycled.update_stamp_reset）
        """
        block_obj = self.blocks[block_name][0]
        block_pos = block_obj.get_pose().p
        if block_pos[2] > self.on_table_z_max:
            self.block_lifted[block_name] = True

    # ------------------------------------------------------------------ #
    #  check_success：状态机主体                                           #
    # ------------------------------------------------------------------ #

    def check_success(self):
        # 0. 生命周期保护：确保 load_actors 已执行
        if not hasattr(self, 'stage_id') or not hasattr(self, 'move_sequence') or not hasattr(self, 'blocks'):
            return False

        # 1. 永久失败短路
        if self.fail_flag:
            return False

        # 2. 更新所有方块的"抬起"磁滞状态
        for block_name in self.block_on_pad:
            self.update_block_lifted(block_name)

        # 3. 画蛇添足检测（白名单模式）：已完成全部 12 步后
        #    任何方块被抬起后的落地行为一律判为永久失败
        if self.stage_id == self.target_move_count:
            for block_name, expected_pad in self.block_on_pad.items():
                if self.block_lifted[block_name]:
                    for pad_id in range(3):
                        if self.check_is_on_pad(block_name, pad_id):
                            reason = (
                                f"放回了原位 pad{pad_id}"
                                if pad_id == expected_pad
                                else f"被多余放置到 pad{pad_id}"
                            )
                            flushed_print(
                                f"❌ 失败：画蛇添足！{block_name} 在任务完成后{reason}。"
                            )
                            self.fail_flag = True
                            return False

            # 终态验证：双手张开 且 双臂回到 back_to_origin 目标点附近 → 任务成功
            # 直接与 robot.left/right_original_pose 比较欧氏距离，避免固定 Z 阈值失效
            left_ee = np.array(self.robot.get_left_ee_pose()[:3])
            right_ee = np.array(self.robot.get_right_ee_pose()[:3])
            left_origin = np.array(self.robot.left_original_pose[:3])
            right_origin = np.array(self.robot.right_original_pose[:3])
            reset_threshold = 0.08  # 单位：米，与 pad_xy_threshold 同量级

            left_reset = np.linalg.norm(left_ee - left_origin) < reset_threshold
            right_reset = np.linalg.norm(right_ee - right_origin) < reset_threshold

            if self.is_left_gripper_open() and self.is_right_gripper_open() and left_reset and right_reset:
                if hasattr(self, 'max_reward'):
                    self.max_reward = max(self.max_reward, 1.0)
                if not self.final_success_printed:
                    flushed_print("✓ 任务成功: 已完成所有方块循环移动，且双臂已复位。")
                    self.final_success_printed = True
                return True
            return False

        elif self.stage_id > self.target_move_count:
            # 防御性检查（正常情况下不应到达此处）
            self.fail_flag = True
            return False

        # 4. 取出当前步期望的移动
        expected_block, from_pad, to_pad = self.move_sequence[self.stage_id]

        # 5. 正确移动判定
        #    条件：期望方块已被抬起（保证是"主动移动"而非初始静置）
        #          且已落在期望目标 pad 上
        if self.check_is_on_pad(expected_block, to_pad) and self.block_lifted[expected_block]:
            flushed_print(
                f"✓ 阶段通过: {expected_block} (pad{from_pad} → pad{to_pad}) "
                f"| 进度: {self.stage_id + 1}/{self.target_move_count}"
            )
            self.block_on_pad[expected_block] = to_pad
            self.block_lifted[expected_block] = False  # 本次落地已确认，重置磁滞
            self.stage_id += 1
            if hasattr(self, 'max_reward'):
                self.max_reward = max(self.max_reward, self.stage_id / float(self.target_move_count))
            return False

        # 6. 错误落地检测（白名单模式）
        #    对所有"已被抬起"的方块，一旦检测到它落在了任意 pad 上：
        #    ▸ 如果它是 expected_block 且落在 to_pad → 正确（步骤 5 已处理）
        #    ▸ 其余一切落地情况（包括放回原处）→ 永久失败
        for block_name in self.block_on_pad:
            if not self.block_lifted[block_name]:
                continue  # 未被抬起，不做检测
            for pad_id in range(3):
                if self.check_is_on_pad(block_name, pad_id):
                    # 唯一允许的落地：期望方块落在期望目标
                    if block_name == expected_block and pad_id == to_pad:
                        pass  # 正确落点（步骤 5 已处理，此处跳过）
                    else:
                        reason = (
                            f"放回了原位 pad{pad_id}"
                            if pad_id == self.block_on_pad[block_name]
                            else f"落在了错误位置 pad{pad_id}"
                        )
                        flushed_print(
                            f"❌ 失败：{block_name} 被抬起后{reason}，"
                            f"但期望是 {expected_block} 移动到 pad{to_pad}。"
                        )
                        self.fail_flag = True
                        return False

        return False

    def _get_current_pads(self):
        """返回 {block_name: pad_idx}，不在任何 pad 附近则返回 -1（调试用）"""
        result = {}
        for name, data in self.blocks.items():
            block_obj = data[0]
            block_pos = block_obj.get_pose().p
            found_pad = -1
            for i, pad_pos in enumerate(self.pad_poses):
                if np.linalg.norm(block_pos[:2] - np.array(pad_pos[:2])) < self.pad_xy_threshold:
                    found_pad = i
                    break
            result[name] = found_pad
        return result

if __name__ == "__main__":
    pass