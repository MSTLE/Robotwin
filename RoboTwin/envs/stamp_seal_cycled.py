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
            "task": "stamp_seal_cycled",
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

        # init signals
        self.stamp_cycled_success_times = 0

        flushed_print("安全工作区初始化完成。")

    def play_once(self):
        flushed_print("开始环境演示...")
        arm_R = ArmTag("right")
        robot_ee_quat = [0.5, -0.5, 0.5, 0.5]

        pre_grasp_dis = 0.1

        # step 1: 抓取印章
        flushed_print("步骤 1: 抓取印章")
        seal_pose_before = self.seal.get_pose()
        self.move(self.grasp_actor(self.seal, arm_tag=arm_R, pre_grasp_dis=pre_grasp_dis))
        self.move(self.move_by_displacement(arm_R, z=0.08))
        if self.seal.get_pose().p[2] - seal_pose_before.p[2] > 0.05:
            flushed_print(f"✓ 印章抓取成功。")
        else:
            flushed_print(f"✗ 印章抓取失败。")
            return self.info

        for i in range(2):
            stamp_success_times = 0
            for j, target_pad_idx in enumerate(self.pad_sequence):
                flushed_print(f"\n------------ 第 {i+1} 循环 第 {j+1} 次 -------------")
                flushed_print(f"开始盖章目标垫子 {target_pad_idx}")

                target_pad = self.pads[target_pad_idx - 1]
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
                z_success = False
                while not z_success:
                    if self.seal.get_pose().p[2] - target_pad_pose.p[2] < 0.004:
                        z_success = True
                    elif self.seal.get_pose().p[2] - target_pad_pose.p[2] > 0.016:
                        self.move(self.move_by_displacement(arm_R, z=-0.01))
                    else:
                        self.move(self.move_by_displacement(arm_R, z=-0.005))
                if np.linalg.norm(np.array(self.seal.get_pose().p) - np.array(target_pad_pose.p)) > 0.015:
                    flushed_print(f"✗ 在垫子 {target_pad_idx} 上盖章失败。")
                    return self.info
                else:
                    stamp_success_times += 1
                    flushed_print(f"✓ 成功在垫子 {target_pad_idx} 上盖章。当前循环已成功盖章次数: {stamp_success_times}")

                # step 4: 抬起印章
                flushed_print("步骤 4: 抬起印章")
                self.move(self.move_by_displacement(arm_R, z=0.1))

            self.stamp_cycled_success_times += 1
            flushed_print(f"\n=== 已完成第 {self.stamp_cycled_success_times} 次完整盖章循环 ===")

        flushed_print("\n环境演示结束。")

    def check_success(self):
        if self.stamp_cycled_success_times == 2:
            flushed_print("✓ 任务成功：已完成两次完整盖章循环。")
            return True
        flushed_print(f"✗ 任务未完成：尚未完成两次完整盖章循环。完成次数：{self.stamp_cycled_success_times}")
        return False

if __name__ == "__main__":
    pass