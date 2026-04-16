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
        self.stage_id = 0
        self.total_stages = 3
        self.fail_flag = False
        self.max_reward = 0.0
        
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

        # stage 4: 复位
        flushed_print("阶段 4: 复位")
        self.move(self.open_gripper(arm_R))
        self.move(self.back_to_origin(arm_R))
        
        flushed_print("演示结束。")
        return self.info

    def _is_right_arm_reset(self):
        """检查右臂是否已复位"""
        right_ee = np.array(self.robot.get_right_ee_pose()[:3])
        right_origin = np.array(self.robot.right_original_pose[:3])
        reset_threshold = 0.08  # 单位：米
        return np.linalg.norm(right_ee - right_origin) < reset_threshold and self.is_right_gripper_open()

    def check_success(self):
        # 0. 生命周期保护
        if not hasattr(self, 'stage_id') or not hasattr(self, 'bell'):
            return False

        # 1. 永久失败检查
        if getattr(self, 'fail_flag', False):
            return False

        # 2. 检查接触状态（参考 click_bell 的精确检测方式）
        is_contact = False
        bell_pose = self.bell.get_contact_point(0)[:3]
        positions = self.get_gripper_actor_contact_position("050_bell")
        eps = [0.025, 0.025]
        for position in positions:
            if (np.all(np.abs(position[:2] - bell_pose[:2]) < eps) and abs(position[2] - bell_pose[2]) < 0.03):
                is_contact = True
                break

        # 3. 总体成功与"画蛇添足"判定
        if self.stage_id == self.total_stages:
            if is_contact:
                flushed_print("❌ 失败：画蛇添足！检测到多余的按铃动作。")
                self.fail_flag = True
                return False
                
            if self._is_right_arm_reset():
                if hasattr(self, 'max_reward'):
                    self.max_reward = max(self.max_reward, 1.0)
                if not self.final_success_printed:
                    flushed_print("✓ 任务整体成功！(短按-长按-短按 节奏完成且机械臂已复位)")
                    self.final_success_printed = True
                return True
            return False
            
        elif self.stage_id > self.total_stages:
            self.fail_flag = True
            return False

        # 4. 追踪节奏（状态机）
        if is_contact:
            self.contact_duration += 1
            self.is_pressed = True
        else:
            if self.is_pressed:  # 刚刚松开
                self.is_pressed = False
                self._evaluate_press(self.contact_duration)
                self.contact_duration = 0

        return False

    def _evaluate_press(self, duration):
        # TODO: 数据由 ai 计算，待验证
        # 基于 DELAY_BASE_TIME = 1
        # 短按：约 427 步 (delay(1) + 移动)
        # 长按：约 3200 步 (delay(10) + 移动)
        
        short_min = 200
        short_max = 600
        long_min = 2500
        long_max = 3800

        if self.stage_id == 0:
            if short_min <= duration <= short_max:
                self.stage_id += 1
                if hasattr(self, 'max_reward'):
                    self.max_reward = max(self.max_reward, self.stage_id / float(self.total_stages))
                flushed_print(f"第一阶段（短按）: ✓ 成功 (持续 {duration} 步)")
            else:
                flushed_print(f"❌ 失败：按压时间错误！第一阶段预期短按持续 {short_min}-{short_max} 步，实际持续 {duration} 步")
                self.fail_flag = True
                
        elif self.stage_id == 1:
            if long_min <= duration <= long_max:
                self.stage_id += 1
                if hasattr(self, 'max_reward'):
                    self.max_reward = max(self.max_reward, self.stage_id / float(self.total_stages))
                flushed_print(f"第二阶段（长按）: ✓ 成功 (持续 {duration} 步)")
            else:
                flushed_print(f"❌ 失败：按压时间错误！第二阶段预期长按持续 {long_min}-{long_max} 步，实际持续 {duration} 步")
                self.fail_flag = True
                
        elif self.stage_id == 2:
            if short_min <= duration <= short_max:
                self.stage_id += 1
                if hasattr(self, 'max_reward'):
                    self.max_reward = max(self.max_reward, self.stage_id / float(self.total_stages))
                flushed_print(f"第三阶段（短按）: ✓ 成功 (持续 {duration} 步)")
            else:
                flushed_print(f"❌ 失败：按压时间错误！第三阶段预期短按持续 {short_min}-{short_max} 步，实际持续 {duration} 步")
                self.fail_flag = True

if __name__ == "__main__":
    pass