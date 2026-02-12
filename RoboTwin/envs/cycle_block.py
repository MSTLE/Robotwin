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
            "task": "cycle_block",
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
            [0.0, -0.050, pad_z],    # pad 0
            [-0.07, -0.15, pad_z],    # pad 1
            [0.07, -0.15, pad_z]      # pad 2
        ]
        self.pads = []

        from .utils.create_actor import create_visual_box
        for i, pos in enumerate(self.pad_poses):
            flushed_print(f"Pad {i} 位置: {pos}")
            self.pad = create_visual_box(
                scene=self.scene,
                pose=sapien.Pose(self.pad_poses[i]),
                half_size=(0.04, 0.04, 0.001),
                color=[45/255, 173/255, 232/255],  # blue
                name=f"pad{i}",
            )
            self.pads.append(self.pad)
        
        # step2: set 2 blocks randomly on the pads
        block_size = 0.02
        block_z = base_table_height + block_size
        block_positions = [
            ([0.0, -0.050, block_z], "block0"),
            ([-0.07, -0.15, block_z], "block1"),
            ([0.07, -0.15, block_z], "block2")
        ]

        block_index = 1

        self.selected_indices = [block_index, (block_index + 1) % 3]

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

        self.empty_pad = (block_index + 2) % 3

        # 初始化状态追踪
        self.move_count = 0        # 成功移动次数
        self.target_move_count = 12 # 目标移动次数 (2轮 * 3次 * 2个方块)
        
        # 记录每个方块的上一次稳定所在的 Pad 索引
        # 初始时需要计算一下
        self.block_last_pad = {} 
        # 我们会在第一次 check_success 或者初始化时填充这个
        
        # 用于状态机：检测方块是否被抓起
        self.held_block = None 
        
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
        arm_L = ArmTag("left")
        arm_R = ArmTag("right")
        change_arm = False

        # 0 -> 1: arm_L
        # 1 -> 2: arm_R
        # 2 -> 0: arm_R
        self.arm_before = None
        arm_use = None
        for i in range(2):
            # # step1: reset some signals
            self.success_catch_times = 0
            self.success_place_times = 0
            

            pre_grasp_dis = 0.1
            grasp_dis=0.03
            MOVE_UP_AFTER_GRASP = 0.05
            robot_quat = [0.5, -0.5, 0.5, 0.5]
            place_down_distance = 0.081

            for j in range(3):
                # catch first block
                first_block, pos_id = self.blocks[f"block{self.selected_indices[1]}"]
                self.arm_before = arm_use
                if pos_id == 0:
                    arm_use = arm_L
                else:
                    arm_use = arm_R
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

                # catch second block
                second_block, pos_id = self.blocks[f"block{self.selected_indices[0]}"]
                self.arm_before = arm_use
                if pos_id == 0:
                    arm_use = arm_L
                else:
                    arm_use = arm_R
                if self.arm_before is not None and self.arm_before != arm_use:
                    change_arm = True
                
                self.move(self.grasp_actor(second_block, arm_tag=arm_use, pre_grasp_dis=pre_grasp_dis, grasp_dis=grasp_dis),
                          self.back_to_origin(self.arm_before) if change_arm else None)
                change_arm = False
                self.move(self.move_by_displacement(arm_use, z=MOVE_UP_AFTER_GRASP))
                
                # 更新内部追踪
                self.blocks[f"block{self.selected_indices[0]}"][1] += 1
                self.blocks[f"block{self.selected_indices[0]}"][1] %= 3
                
                # place second block
                target_pos_pre = self.pad_poses[self.empty_pad].copy()
                target_pos_pre[2] += 0.25
                self.move(self.move_to_pose(arm_use, sapien.Pose(target_pos_pre, robot_quat)))
                self.move(self.move_by_displacement(arm_use, z=-place_down_distance))
                self.move(self.open_gripper(arm_use))
                self.move(self.move_by_displacement(arm_use, z=MOVE_UP_AFTER_GRASP))
                
                # 更新空Pad索引
                self.empty_pad = (self.empty_pad + 2) % 3
                
            flushed_print(f"=== 第 {i+1} 轮循环演示完成 ===")

        return self.info

    def check_success(self):
        # 0. 防止初始化前运行
        if not hasattr(self, 'blocks') or not hasattr(self, 'pad_poses'):
            return False

        # 1. 首次运行时，初始化 block 所在 Pad
        if not self.block_last_pad:
            self._update_all_blocks_pad()
            
        # 2. 定期全量检测是否有方块位移
        # 只有当方块不在手上时检测最稳
        # 这里简化：检测所有方块目前的 Pad
        current_pads = self._get_current_pads()
        
        for block_name, current_pad in current_pads.items():
            last_pad = self.block_last_pad.get(block_name, -1)
            
            # 如果方块发生了移动（从一个Pad到了另一个Pad）
            if current_pad != -1 and current_pad != last_pad:
                 if last_pad != -1: # 确保是从一个Pad移动过来的，而不是第一次检测
                    self.move_count += 1
                    flushed_print(f"✓ 有效移动: {block_name} (Pad {last_pad} -> Pad {current_pad}) | 总计: {self.move_count}/{self.target_move_count}")
                 
                 # 更新状态
                 self.block_last_pad[block_name] = current_pad

        # 3. 总体成功判定
        if self.move_count >= self.target_move_count:
            if not self.final_success_printed:
                flushed_print("✓ 任务成功: 已完成所有方块循环移动。")
                self.final_success_printed = True
            return True
            
        return False
        
    def _update_all_blocks_pad(self):
        """初始化所有 block 的位置"""
        current_pads = self._get_current_pads()
        self.block_last_pad = current_pads
        
    def _get_current_pads(self):
        """返回 {block_name: pad_idx}，如果不在任何 pad 附近返回 -1"""
        result = {}
        for name, data in self.blocks.items():
            block_obj = data[0]
            block_pos = block_obj.get_pose().p
            
            found_pad = -1
            for i, pad_pos in enumerate(self.pad_poses):
                # XY 平面距离 < 4cm
                if np.linalg.norm(block_pos[:2] - np.array(pad_pos[:2])) < 0.04:
                    found_pad = i
                    break
            result[name] = found_pad
        return result

if __name__ == "__main__":
    pass