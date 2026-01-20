from ._base_task import Base_Task
from .utils import *
import sapien
import numpy as np
import sys

def flushed_print(*args, **kwargs):
    """确保日志能够实时刷新的打印函数"""
    print(*args, **kwargs)
    sys.stdout.flush()

class put_back_block(Base_Task):

    def setup_demo(self, **kwags):
        super()._init_task_env_(**kwags)

    def load_actors(self):
        flushed_print("正在加载资产...")
        
        # === 位置配置区 (你可以直接修改这里的数值) ===
        # A 点随机范围 (左侧抓取点)
        self.config_A = {
            "xlim": [-0.4, -0.15],
            "ylim": [-0.2, -0.15],
            "z": 0.77
        }
        # B 点固定坐标 (中央中转点)
        self.config_B = {
            "pos": [-0.1, 0.0, 0.77]
        }
        # 铃铛/按钮固定坐标 (右侧按压点)
        self.config_bell = {
            "pos": [0.3, 0.0, 0.77],
            "quat": [0.707, 0.707, 0, 0] # 指向前方
        }
        # ========================================

        # 1. 初始位置 A (左后方)
        self.pos_A_init_pose = rand_pose(
            xlim=self.config_A["xlim"],
            ylim=self.config_A["ylim"],
            zlim=[self.config_A["z"]],
            rotate_rand=True,
            rotate_lim=[0, 0, 0.2],
        )
        
        self.block = create_box(
            scene=self,
            pose=self.pos_A_init_pose,
            half_size=(0.02, 0.02, 0.02),
            color=(1, 0, 0),
            name="block",
            is_static=False,
        )
        self.block.set_mass(0.01)

        # 2. 目标位置 B (中央后方)
        self.pos_B_pose = sapien.Pose(self.config_B["pos"])
        
        # 3. 铃铛 (右后方)
        bell_pos = sapien.Pose(self.config_bell["pos"], self.config_bell["quat"])
        self.bell = create_actor(
            scene=self,
            pose=bell_pos,
            modelname="050_bell",
            model_id=1,
            convex=True,
            is_static=True,
        )
        
        self.bell_clicked = False
        flushed_print("安全工作区初始化完成。")

    def play_once(self):
        flushed_print("执行 '放回方块' 任务序列...")
        
        # --- 步骤 1: 左臂从 A 点抓取方块 ---
        arm_L = ArmTag("left")
        flushed_print(f"左臂: 正在从 A 点抓取方块")
        self.move(self.grasp_actor(self.block, arm_tag=arm_L, pre_grasp_dis=0.1))
        
        # 抓取后抬高，避免 Step 2 放置时姿态突变
        self.move(self.move_by_displacement(arm_L, z=0.15))
        
        # --- 步骤 2: 左臂将方块放置在 B 点 (中转站) ---
        flushed_print(f"左臂: 正在放置到 B 点 (中转站)")
        
        # 先移动到 B 点上方，保持当前姿态
        current_q = self.robot.get_left_ee_pose()[3:]
        target_B_high = self.config_B["pos"].copy()
        target_B_high[2] += 0.12
        self.move(self.move_to_pose(arm_L, target_B_high + current_q))
        
        # 垂直下降到放置高度（保持姿态不变）
        self.move(self.move_by_displacement(arm_L, z=-0.10))
        
        # 打开夹爪释放方块
        self.move(self.open_gripper(arm_L))
        
        # 垂直抬起
        self.move(self.move_by_displacement(arm_L, z=0.10))
        
        # 回原点
        self.move(self.back_to_origin(arm_L))
        
        if not self.plan_success:
            flushed_print("失败: 左臂任务执行失败")
            return self.info

        # --- 步骤 3: 右臂去点铃铛 ---
        arm_R = ArmTag("right")
        flushed_print(f"右臂: 正在点铃铛")
        press_pre = self.get_grasp_pose(self.bell, pre_dis=0.1, contact_point_id=0, arm_tag=arm_R)
        if press_pre:
            # 设置姿态指向下方
            press_pre[3:] = [0.5, -0.5, 0.5, 0.5]
            self.move(self.move_to_pose(arm_R, press_pre))
            self.move(self.close_gripper(arm_R))
            self.move(self.move_by_displacement(arm_R, z=-0.07))
            self.bell_clicked = True
            self.move(self.move_by_displacement(arm_R, z=0.07))
            self.move(self.back_to_origin(arm_R))
        else:
            flushed_print("失败: 铃铛点击路径规划失败")
            self.plan_success = False
            return self.info

        # --- 步骤 4: 左臂从 B 点抓取方块 ---
        flushed_print(f"左臂: 正在从 B 点抓取方块")
        self.move(self.grasp_actor(self.block, arm_tag=arm_L, pre_grasp_dis=0.05, contact_point_id=0))
        
        if not self.plan_success:
            flushed_print("失败: 左臂从 B 点抓取规划失败")
            return self.info
        
        # 抓取后立即抬高，避免姿态扭曲累积
        flushed_print(f"左臂: 抬高方块")
        self.move(self.move_by_displacement(arm_L, z=0.15))

        # --- 步骤 5: 左臂将方块放回 A 点 ---
        flushed_print(f"左臂: 正在将方块放回 A 点")
        
        # 移动到 A 点上方，保持当前姿态
        current_q = self.robot.get_left_ee_pose()[3:]
        target_A_high = self.pos_A_init_pose.p.tolist()
        target_A_high[2] += 0.12
        self.move(self.move_to_pose(arm_L, target_A_high + current_q))
        
        if not self.plan_success:
            flushed_print("失败: 移动到 A 点上方规划失败")
            return self.info
        
        # 垂直下降到放置高度
        self.move(self.move_by_displacement(arm_L, z=-0.10))
        
        # 打开夹爪释放方块
        self.move(self.open_gripper(arm_L))
        
        # 垂直抬起
        self.move(self.move_by_displacement(arm_L, z=0.10))
        
        # 回原点
        self.move(self.back_to_origin(arm_L))

        if self.plan_success:
            flushed_print("成功: 专家演示完成")
        else:
            flushed_print("失败: 最终放置阶段失败")
        return self.info

    def check_success(self):
        block_p = self.block.get_pose().p
        init_p = self.pos_A_init_pose.p
        dist = np.linalg.norm(block_p[:2] - init_p[:2])
        # 成功条件: 方块回到 A 点且铃铛已被点过
        success = dist < 0.05 and self.bell_clicked
        flushed_print(f"成功检查: 距离={dist:.3f}, 铃铛点击={self.bell_clicked} -> {success}")
        return success

if __name__ == "__main__":
    pass
