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

        # init signals
        
        self.success_cycle_times = 0
        self.success_catch_times = 0
        self.success_place_times = 0

        flushed_print("安全工作区初始化完成。")

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
                start_block_pos = first_block.get_pose().p
                self.move(self.grasp_actor(first_block, arm_tag=arm_use, pre_grasp_dis=pre_grasp_dis, grasp_dis=grasp_dis),
                          self.back_to_origin(self.arm_before) if change_arm else None)
                change_arm = False
                self.move(self.move_by_displacement(arm_use, z=MOVE_UP_AFTER_GRASP))
                end_block_pos = first_block.get_pose().p
                # check catch success
                flushed_print(f"DEBUG: 第 {i+1} 轮第 1 个方块第 {j+1} 次 抓取前位置: {start_block_pos}, 抓取后位置: {end_block_pos}")
                if (end_block_pos - start_block_pos)[2] > MOVE_UP_AFTER_GRASP - 0.01:
                    self.success_catch_times += 1
                    self.blocks[f"block{self.selected_indices[1]}"][1] += 1
                    self.blocks[f"block{self.selected_indices[1]}"][1] %= 3
                    flushed_print(f"✓ 第 {i+1} 轮第 1 个方块第 {j+1} 次 抓取 方块成功 ")
                else:
                    flushed_print(f"✗ 第 {i+1} 轮第 1 个方块第 {j+1} 次 抓取 方块失败 ")
                    return self.info
                
                # place first block
                target_pos_pre = self.pad_poses[self.empty_pad].copy()
                target_pos_pre[2] += 0.25
                flushed_print(f"DEBUG: 第 {i+1} 轮第 1 个方块第 {j+1} 次 放置 目标位置: {target_pos_pre}")
                self.move(self.move_to_pose(arm_use, sapien.Pose(target_pos_pre, robot_quat)))

                self.move(self.move_by_displacement(arm_use, z=-place_down_distance))
                self.move(self.open_gripper(arm_use))
                self.move(self.move_by_displacement(arm_use, z=MOVE_UP_AFTER_GRASP))
                # check place success
                block_p = first_block.get_pose().p
                if np.linalg.norm(block_p[:2] - self.pad_poses[self.empty_pad][:2]) < 0.03:
                    self.success_place_times += 1
                    flushed_print(f"✓ 第 {i+1} 轮第 1 个方块第 {j+1} 次 放置 方块成功 ")
                    self.empty_pad = (self.empty_pad + 2) % 3
                else:
                    flushed_print(f"✗ 第 {i+1} 轮第 1 个方块第 {j+1} 次 放置 方块失败 ")
                    return self.info

                # catch second block
                second_block, pos_id = self.blocks[f"block{self.selected_indices[0]}"]
                self.arm_before = arm_use
                if pos_id == 0:
                    arm_use = arm_L
                else:
                    arm_use = arm_R
                if self.arm_before is not None and self.arm_before != arm_use:
                    change_arm = True
                start_block_pos = second_block.get_pose().p
                self.move(self.grasp_actor(second_block, arm_tag=arm_use, pre_grasp_dis=pre_grasp_dis, grasp_dis=grasp_dis),
                          self.back_to_origin(self.arm_before) if change_arm else None)
                change_arm = False
                self.move(self.move_by_displacement(arm_use, z=MOVE_UP_AFTER_GRASP))
                end_block_pos = second_block.get_pose().p
                # check catch success
                flushed_print(f"DEBUG: 第 {i+1} 轮第 2 个方块第 {j+1} 次 抓取前位置: {start_block_pos}, 抓取后位置: {end_block_pos}")
                if (end_block_pos - start_block_pos)[2] > MOVE_UP_AFTER_GRASP - 0.01:
                    self.success_catch_times += 1
                    self.blocks[f"block{self.selected_indices[0]}"][1] += 1
                    self.blocks[f"block{self.selected_indices[0]}"][1] %= 3
                    flushed_print(f"✓ 第 {i+1} 轮第 2 个方块第 {j+1} 次 抓取 方块成功 ")
                else:
                    flushed_print(f"✗ 第 {i+1} 轮第 2 个方块第 {j+1} 次 抓取 方块失败 ")
                    return self.info

                # place second block
                target_pos_pre = self.pad_poses[self.empty_pad].copy()
                target_pos_pre[2] += 0.25
                flushed_print(f"DEBUG: 第 {i+1} 轮第 2 个方块第 {j+1} 次 放置 目标位置: {target_pos_pre}")
                self.move(self.move_to_pose(arm_use, sapien.Pose(target_pos_pre, robot_quat)))
                self.move(self.move_by_displacement(arm_use, z=-place_down_distance))
                self.move(self.open_gripper(arm_use))
                self.move(self.move_by_displacement(arm_use, z=MOVE_UP_AFTER_GRASP))
                block_p = second_block.get_pose().p
                if np.linalg.norm(block_p[:2] - self.pad_poses[self.empty_pad][:2]) < 0.03:
                    self.success_place_times += 1
                    flushed_print(f"✓ 第 {i+1} 轮第 2 个方块第 {j+1} 次 放置 方块成功 ")
                    self.empty_pad = (self.empty_pad + 2) % 3
                else:
                    flushed_print(f"✗ 第 {i+1} 轮第 2 个方块第 {j+1} 次 放置 方块失败 ")
                    return self.info
                
            self.success_cycle_times += 1

        return self.info

    def check_success(self):
        if self.success_cycle_times == 2:
            flushed_print("✓ 两轮循环成功完成")
            return True
        flushed_print("✗ 循环未成功完成")
        return False

if __name__ == "__main__":
    pass