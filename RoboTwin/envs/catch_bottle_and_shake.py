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

        # signals
        self.catch_success = False
        self.shake_success_times = 0

        flushed_print("安全工作区初始化完成。")
    
    def play_once(self):
        flushed_print("执行 '放回方块' 任务序列（三次循环）...")

        arm_R = ArmTag("right")

        # step 1: 抓取瓶子
        flushed_print("步骤 1: 抓取瓶子")

        start_p = self.bottle.get_pose().p
        self.move(self.grasp_actor(self.bottle, arm_tag=arm_R, pre_grasp_dis=0.15))
        self.move(self.move_by_displacement(arm_R, z=0.1))
        end_p = self.bottle.get_pose().p

        flushed_print(f"DEBUG: 初始高度 {start_p[2]:.4f}")
        flushed_print(f"DEBUG: 判定阈值 {(start_p[2] + 0.08):.4f}")
        flushed_print(f"DEBUG: 当前检测高度 {self.bottle.get_pose().p[2]:.4f}")

        move_z = np.linalg.norm(np.array(end_p[2]) - np.array(start_p[2]))
        if move_z > 0.08:
            flushed_print("✓ 瓶子抓取成功。")
            self.catch_success = True
        else:
            flushed_print("✗ 瓶子抓取失败。")

        # step 2: 摇晃瓶子
        flushed_print("步骤 2: 摇晃瓶子")
        for cycle in range(3):
            flushed_print(f"\n{'='*40}")
            flushed_print(f"开始第 {cycle + 1} 次循环")
            flushed_print(f"{'='*40}")

            # reset signals
            shake_up_success = False
            shake_down_success = False
            
            # shake up
            start_p = self.bottle.get_pose().p
            self.move(self.move_by_displacement(arm_R, z=0.15))
            end_p = self.bottle.get_pose().p
            move_z = np.linalg.norm(np.array(end_p[2]) - np.array(start_p[2]))
            if move_z > 0.13:
                flushed_print("摇晃上动作成功。")
                shake_up_success = True
            else:
                flushed_print("摇晃上动作失败。")
                flushed_print(f"瓶子位移: {move_z:.3f} 米")

            # shake down
            start_p = self.bottle.get_pose().p
            self.move(self.move_by_displacement(arm_R, z=-0.15))
            end_p = self.bottle.get_pose().p
            move_z = np.linalg.norm(np.array(end_p[2]) - np.array(start_p[2]))
            if move_z > 0.13:
                flushed_print("摇晃下动作成功。")
                shake_down_success = True
            else:
                flushed_print("摇晃下动作失败。")
                flushed_print(f"瓶子位移: {move_z:.3f} 米")

            # check success
            if shake_up_success and shake_down_success:
                flushed_print(f"第 {cycle + 1} 次循环摇晃成功。")
                self.shake_success_times += 1
            else:
                flushed_print(f"第 {cycle + 1} 次循环摇晃失败。")
        
        # move the arm back
        # self.move(self.back_to_origin(arm_R))

        return self.info

    def check_success(self):
        overall_success = False
        flushed_print("\n========== 成功检查 ==========")

        flushed_print(f"抓取瓶子：{'✓ 成功' if self.catch_success else '✗ 失败'}")
        
        if self.shake_success_times == 3:
            flushed_print("摇晃瓶子: ✓ 成功")
            overall_success = True if self.catch_success else False
        elif self.shake_success_times > 0:
            flushed_print(f"摇晃瓶子: ✗ 部分成功 ({self.shake_success_times}/3)")
        else:
            flushed_print("摇晃瓶子: ✗ 失败")

        flushed_print(f"\n任务整体结果: {'✓ 成功' if overall_success else '✗ 失败'}")
        flushed_print("================================\n")

        return overall_success

        
if __name__ == "__main__":
    pass