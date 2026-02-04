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

class ring_bell_rhythm(Base_Task):
    """铃声节奏任务环境"""

    def setup_demo(self, **kwags):
        super()._init_task_env_(**kwags)

        self.info["info"] = {
            "task": "ring_bell_rhythm",
            "{a}": "right",
            "{A}": "050_bell/base1"
        }

    def load_actors(self):
        flushed_print("正在加载资产...")
        base_table_height = 0.74

        # ----- delay_time (can be change) ----
        self.DELAY_BASE_TIME = 20
        # -------------------------------------

        # load bell
        self.config_bell = {
            "pos": [0.1, -0.1, base_table_height + 0.05],
            "quat": [0.707, 0.707, 0.0, 0.0] # 指向上方
        }
        self.bell = create_actor(
            scene=self,
            pose=sapien.Pose(self.config_bell["pos"], self.config_bell["quat"]),
            modelname="050_bell",
            model_id=1,
            convex=True,
            is_static=True,
        )

        # init signal
        self.stage1_success = False  # 短按
        self.stage2_success = False  # 长按
        self.stage3_success = False  # 短按

        self.bell_clicked = False

        flushed_print("安全工作区初始化完成。")

    def play_once(self):
        flushed_print("开始环境演示...")
        arm_R = ArmTag("right")
        robot_ee_quat = [0.707, 0.707, 0.0, 0.0]  # 指向上方

        press_pre = self.get_grasp_pose(self.bell, pre_dis=0.1, contact_point_id=0, arm_tag=arm_R)
        if press_pre:
            # 设置姿态指向下方
            press_pre[3:] = [0.5, -0.5, 0.5, 0.5]
            self.move(self.move_to_pose(arm_R, press_pre))
            self.move(self.close_gripper(arm_R))
            self.bell_clicked = True
            flushed_print("✓ 成功: 铃铛点击路径规划成功")
        else:
            flushed_print(f"✗ 失败: 铃铛点击路径规划失败")
            return self.info
        
        move_down_distance = 0.038

        # stage 1: 短按
        flushed_print("阶段 1: 短按铃铛")
        self.move(self.move_by_displacement(arm_R, z=-move_down_distance))
        # flushed_print(f"DEBUG: time delay before: {time.perf_counter()}")
        self.delay(int(self.DELAY_BASE_TIME))
        # flushed_print(f"DEBUG: time delay after: {time.perf_counter()}")
        self.move(self.move_by_displacement(arm_R, z=move_down_distance))
        self.stage1_success = True

        # stage 2: 长按
        flushed_print("阶段 2: 长按铃铛")
        self.move(self.move_by_displacement(arm_R, z=-move_down_distance))
        self.delay(int(self.DELAY_BASE_TIME * 2.5))
        self.move(self.move_by_displacement(arm_R, z=move_down_distance))
        self.stage2_success = True

        # stage 3: 短按
        flushed_print("阶段 3: 短按铃铛")
        self.move(self.move_by_displacement(arm_R, z=-move_down_distance))
        self.delay(int(self.DELAY_BASE_TIME))
        self.move(self.move_by_displacement(arm_R, z=move_down_distance))
        self.stage3_success = True

        flushed_print("演示结束。")
        return self.info

    def check_success(self):
        flushed_print(f"第一阶段（短按）:{'✓ 成功' if self.stage1_success else '✗ 失败'}")
        flushed_print(f"第二阶段（长按）:{'✓ 成功' if self.stage2_success else '✗ 失败'}")
        flushed_print(f"第三阶段（短按）:{'✓ 成功' if self.stage3_success else '✗ 失败'}")
        success_overall = self.stage1_success and self.stage2_success and self.stage3_success
        if success_overall:
            flushed_print("✓ 总体任务已完成！")
            return True
        else:
            flushed_print("✗ 总体任务未完成。")
            return False

if __name__ == "__main__":
    pass