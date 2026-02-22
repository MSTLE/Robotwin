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
            "{b}": "right",
            "{A}": "050_bell/base1",
            "{B}": "the table"
        }

    def load_actors(self):
        flushed_print("正在加载资产...")
        base_table_height = 0.74

        # ----- delay_time (can be change) ----
        self.DELAY_BASE_TIME = 1
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
        
        self.contact_duration = 0
        self.is_pressed = False
        self.final_success_printed = False

        self.bell_clicked = False

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

            if (left_arm is not None and control_idx < left_arm["position"].shape[0]):  # control left arm
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

            if (right_arm is not None and control_idx < right_arm["position"].shape[0]):  # control right arm
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
        self.delay(int(self.DELAY_BASE_TIME))
        self.move(self.move_by_displacement(arm_R, z=move_down_distance))

        # stage 2: 长按
        flushed_print("阶段 2: 长按铃铛")
        self.move(self.move_by_displacement(arm_R, z=-move_down_distance))
        self.delay(int(self.DELAY_BASE_TIME * 10.0))
        self.move(self.move_by_displacement(arm_R, z=move_down_distance))

        # stage 3: 短按
        flushed_print("阶段 3: 短按铃铛")
        self.move(self.move_by_displacement(arm_R, z=-move_down_distance))
        self.delay(int(self.DELAY_BASE_TIME))
        self.move(self.move_by_displacement(arm_R, z=move_down_distance))

        flushed_print("演示结束。")
        return self.info

    def check_success(self):
        # 防止在初始化完成前进行检查
        if not hasattr(self, 'bell'):
            return False

        # 1. 检查接触状态（参考 click_bell 的精确检测方式）
        is_contact = False
        bell_pose = self.bell.get_contact_point(0)[:3]
        positions = self.get_gripper_actor_contact_position("050_bell")
        eps = [0.025, 0.025]
        for position in positions:
            if (np.all(np.abs(position[:2] - bell_pose[:2]) < eps) and abs(position[2] - bell_pose[2]) < 0.03):
                is_contact = True
                break
        
        # 2. 追踪节奏（状态机）
        if is_contact:
            self.contact_duration += 1
            self.is_pressed = True
        else:
            if self.is_pressed:  # 刚刚松开
                self.is_pressed = False
                self._evaluate_press(self.contact_duration)
                self.contact_duration = 0

        # 3. 总体成功判定
        success_overall = self.stage1_success and self.stage2_success and self.stage3_success
        if success_overall:
            if not self.final_success_printed:
                flushed_print("✓ 总体任务已完成！")
                self.final_success_printed = True
            return True
        else:
            # 在评估期间，在完成前返回 False
            return False

    def _evaluate_press(self, duration):
        # 基于 DELAY_BASE_TIME = 1
        # 短按：约 427 步 (delay(1) + 移动)
        # 长按：约 3200 步 (delay(10) + 移动)
        
        short_min = 200
        short_max = 600
        long_min = 2500
        long_max = 3800

        if not self.stage1_success:
            if short_min <= duration <= short_max:
                self.stage1_success = True
                flushed_print(f"第一阶段（短按）: ✓ 成功 (持续 {duration} 步)")
            else:
                flushed_print(f"按压失败: 持续 {duration} 步 (预期范围 {short_min}-{short_max})")
        elif not self.stage2_success:
            if long_min <= duration <= long_max:
                self.stage2_success = True
                flushed_print(f"第二阶段（长按）: ✓ 成功 (持续 {duration} 步)")
            else:
                flushed_print(f"按压失败: 持续 {duration} 步 (预期范围 {long_min}-{long_max})")
        elif not self.stage3_success:
            if short_min <= duration <= short_max:
                self.stage3_success = True
                flushed_print(f"第三阶段（短按）: ✓ 成功 (持续 {duration} 步)")
            else:
                flushed_print(f"按压失败: 持续 {duration} 步 (预期范围 {short_min}-{short_max})")

if __name__ == "__main__":
    pass