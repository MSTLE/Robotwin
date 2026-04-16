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
        # 填充 episode 信息用于指令生成
        self.info["info"] = {
            "A": "red block",
            "B": "blue square",
            "C": "bell",
            "a": "left",
            "b": "right"
        }

    def load_actors(self):
        flushed_print("正在加载资产...")

        # === 位置配置区 (可以修改这里的数值) ===
        # A 点随机范围
        self.config_A = {
            "xlim": [-0.4, -0.15],
            "ylim": [-0.2, -0.15],
            "z": 0.77
        }
        # B 点固定坐标
        self.config_B = {
            "pos": [-0.1, 0.0, 0.77]
        }
        # 铃铛固定坐标
        self.config_bell = {
            "pos": [0.18, 0.0, 0.77],
            "quat": [0.707, 0.707, 0, 0] # 指向上方
        }
        # ========================================

        # 1. 初始位置 A 
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
        
        # 在 B 点创建一个蓝色正方形标记 (visual marker)
        # B 点坐标: [-0.1, 0.0, 0.77]
        # 创建一个很薄的 box 放在桌面上
        marker_pos = self.config_B["pos"].copy()
        marker_pos[2] = 0.741 # 桌面上方一点点

        # 使用 create_visual_box 创建纯视觉对象
        from .utils.create_actor import create_visual_box
        self.marker_B = create_visual_box(
            scene=self.scene,
            pose=sapien.Pose(marker_pos),
            half_size=[0.04, 0.04, 0.001], # 8cm x 8cm 正方形
            color=[0, 0, 1], # 蓝色
            name="marker_B",
        )

        # 3. 铃铛
        bell_pos = sapien.Pose(self.config_bell["pos"], self.config_bell["quat"])
        self.bell = create_actor(
            scene=self,
            pose=bell_pos,
            modelname="050_bell",
            model_id=1,
            convex=True,
            is_static=True,
        )

        # ---- 状态机初始化（全部在 load_actors 完成）----

        # 任务由 12 个阶段组成（2 次循环 × 6 阶段）
        # stage_id (单次循环):
        # 0: A→B      1: 左臂复位
        # 2: 点铃铛    3: 右臂复位
        # 4: B→A      5: 左臂复位
        self.total_stages = 12
        self.stage_id = 0       # 当前阶段索引 (0 ~ 5)
        self.fail_flag = False  # 永久失败标志
        self.max_reward = 0.0

        # 判定阈值
        self.block_xy_threshold = 0.05   # 方块落点 XY 距离阈值 (m)
        self.block_z_lift_min   = 0.82   # 方块被视为"已抬起"的最低 Z 值 (m)
        self.bell_xy_threshold  = 0.025  # 铃铛接触判定 XY 范围 (m)
        self.bell_z_threshold   = 0.03   # 铃铛接触判定 Z 范围 (m)

        # 磁滞标志：方块是否已被抬起（防止初始静置误触发落地检测）
        self.block_lifted = False

        # 铃铛按压磁滞：已检测到按压后，需抬起才能检测下一次
        self.bell_pressed_flag = False  # True = 正在接触铃铛
        self.bell_lifted_flag  = True   # True = 已离开铃铛（允许触发下次检测）

        # 防重复打印
        self.final_success_printed = False

        flushed_print("安全工作区初始化完成。")

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
        """演示完整任务序列（两次循环，共 6 个阶段）。
        本函数仅包含动作序列，不含任何判断逻辑。
        所有检查逻辑均在 check_success 中完成。
        """
        flushed_print("执行 '放回方块' 任务序列（两次循环）...")
        arm_L = ArmTag("left")
        arm_R = ArmTag("right")

        # 执行两次完整循环
        for cycle in range(2):
            flushed_print(f"\n{'='*40}")
            flushed_print(f"开始第 {cycle + 1} 次循环")
            flushed_print(f"{'='*40}")

            # --- 阶段 1: 左臂从 A 点抓取方块，放至 B 点 ---
            flushed_print(f"左臂: 从 A 点抓取方块")
            self.move(self.grasp_actor(self.block, arm_tag=arm_L, pre_grasp_dis=0.1, grasp_dis=0.03))
            
            # 抓取后抬高，避免 Step 2 放置时姿态突变
            self.move(self.move_by_displacement(arm_L, z=0.15))

            # --- 步骤 2: 左臂将方块放置在 B 点 (中转站) ---
            flushed_print(f"左臂: 正在放置到 B 点 (中转站)")
            
            # 先移动到 B 点上方，保持当前姿态
            current_q = self.robot.get_left_ee_pose()[3:]
            target_B_high = self.config_B["pos"].copy()
            target_B_high[2] += 0.18
            self.move(self.move_to_pose(arm_L, target_B_high + current_q))
            
            # 垂直下降到放置高度（保持姿态不变）
            self.move(self.move_by_displacement(arm_L, z=-0.05))
            
            # 打开夹爪释放方块``
            self.move(self.open_gripper(arm_L))
            
            # 垂直抬起
            self.move(self.move_by_displacement(arm_L, z=0.10))
            
            # 阶段1检查将由 check_success 自动完成
            
            # 回原点
            self.move(self.back_to_origin(arm_L))

            # --- 阶段 2: 右臂点击铃铛 ---
            flushed_print(f"右臂: 点击铃铛")
            press_pre = self.get_grasp_pose(self.bell, pre_dis=0.1, contact_point_id=0, arm_tag=arm_R)
            if press_pre:
                # 设置姿态指向下方
                press_pre[3:] = [0.5, -0.5, 0.5, 0.5]
                self.move(self.move_to_pose(arm_R, press_pre))
                self.move(self.close_gripper(arm_R))
                self.move(self.move_by_displacement(arm_R, z=-0.05))
                self.move(self.move_by_displacement(arm_R, z=0.07))
                self.move(self.open_gripper(arm_R))   # 打开夹爪，确保复位时夹爪张开
                self.move(self.back_to_origin(arm_R))

            # --- 阶段 3: 左臂从 B 点取回方块，放回 A 点 ---
            flushed_print(f"左臂: 从 B 点抓取方块")
            self.move(self.grasp_actor(self.block, arm_tag=arm_L, pre_grasp_dis=0.05, contact_point_id=0, grasp_dis=0.03))
            self.move(self.move_by_displacement(arm_L, z=0.15))

            # --- 步骤 5: 左臂将方块放回 A 点 ---
            flushed_print(f"左臂: 正在将方块放回 A 点")
            
            # 移动到 A 点上方，保持当前姿态
            current_q = self.robot.get_left_ee_pose()[3:]
            target_A_high = self.pos_A_init_pose.p.tolist()
            target_A_high[2] += 0.18
            self.move(self.move_to_pose(arm_L, target_A_high + current_q))
            self.move(self.move_by_displacement(arm_L, z=-0.05))
            
            # 打开夹爪释放方块
            self.move(self.open_gripper(arm_L))
            
            # 垂直抬起
            self.move(self.move_by_displacement(arm_L, z=0.10))
            
            # 阶段3检查将由 check_success 自动完成
            
            # 回原点
            self.move(self.back_to_origin(arm_L))

        flushed_print("\n演示序列结束。")
        return self.info

    # ------------------------------------------------------------------ #
    #  辅助函数                                                            #
    # ------------------------------------------------------------------ #

    def _is_block_on_A(self):
        """检查方块是否静置在 A 点附近（XY 距离 < 阈值 且 Z 在桌面高度）。"""
        block_p = self.block.get_pose().p
        init_p  = self.pos_A_init_pose.p
        xy_dist = np.linalg.norm(block_p[:2] - init_p[:2])
        return xy_dist < self.block_xy_threshold and block_p[2] <= self.block_z_lift_min

    def _is_block_on_B(self):
        """检查方块是否静置在 B 点附近（XY 距离 < 阈值 且 Z 在桌面高度）。"""
        block_p = self.block.get_pose().p
        b_p     = self.pos_B_pose.p
        xy_dist = np.linalg.norm(block_p[:2] - b_p[:2])
        return xy_dist < self.block_xy_threshold and block_p[2] <= self.block_z_lift_min

    def _update_block_lifted(self):
        """磁滞：当方块 Z 超过抬起阈值时，单向置位 block_lifted。"""
        block_z = self.block.get_pose().p[2]
        if block_z > self.block_z_lift_min:
            self.block_lifted = True
        elif block_z < 0.78:
            # 极端严格：如果接触桌面（落地）且不在 A/B 目标区，则判定脱手，强制重新抓取
            if not self._is_block_on_A() and not self._is_block_on_B():
                self.block_lifted = False

    def _is_bell_touched(self):
        """检查夹爪是否正在接触铃铛（通过接触点位置检测）。"""
        try:
            bell_contact = self.bell.get_contact_point(0)[:3]
            positions = self.get_gripper_actor_contact_position("050_bell")
            for pos in positions:
                if (np.all(np.abs(pos[:2] - bell_contact[:2]) < self.bell_xy_threshold) and
                        abs(pos[2] - bell_contact[2]) < self.bell_z_threshold):
                    return True
        except Exception:
            pass
        return False

    def _is_left_arm_reset(self):
        """检查左臂是否已复位"""
        left_ee = np.array(self.robot.get_left_ee_pose()[:3])
        left_origin = np.array(self.robot.left_original_pose[:3])
        reset_threshold = 0.08  # 单位：米
        return np.linalg.norm(left_ee - left_origin) < reset_threshold and self.is_left_gripper_open()

    def _is_right_arm_reset(self):
        """检查右臂是否已复位"""
        right_ee = np.array(self.robot.get_right_ee_pose()[:3])
        right_origin = np.array(self.robot.right_original_pose[:3])
        reset_threshold = 0.08  # 单位：米
        return np.linalg.norm(right_ee - right_origin) < reset_threshold and self.is_right_gripper_open()

    # ------------------------------------------------------------------ #
    #  check_success：状态机主体                                           #
    # ------------------------------------------------------------------ #

    def check_success(self):
        """逐帧检查任务进度。

        状态机（stage_id 0 ~ 11）：
          0: 等待方块被放到 B 点（A→B 第1次）
          1: 等待左臂复位
          2: 等待铃铛被点击（第1次）
          3: 等待右臂复位
          4: 等待方块被放回 A 点（B→A 第1次）
          5: 等待左臂复位
          6-11: 第2次循环，动作同上
          12: 全部完成，等待双臂复位
        """
        # 0. 生命周期保护
        if not hasattr(self, 'stage_id') or not hasattr(self, 'block'):
            return False

        # 1. 永久失败短路
        if self.fail_flag:
            return False

        # 2. 更新方块抬起磁滞状态
        self._update_block_lifted()

        # ==============================================================
        # 3. 已完成全部 12 个阶段：等待双臂复位，同时防"画蛇添足"
        # ==============================================================
        if self.stage_id == self.total_stages:
            # 3a. 防止方块被再次移动（画蛇添足检测）
            if self.block_lifted:
                # 若方块已被抬起后又落到任意位置，视为失败
                if self._is_block_on_A() or self._is_block_on_B():
                    flushed_print("❌ 失败：任务完成后方块被多余移动。")
                    self.fail_flag = True
                    return False

            # 3b. 双臂复位检查
            if (self._is_left_arm_reset() and self._is_right_arm_reset() and self._is_block_on_A()):
                self.max_reward = max(self.max_reward, 1.0)
                if not self.final_success_printed:
                    flushed_print("✓ 任务成功：两次完整循环已完成，方块最终停在 A 点，且双臂已复位。")
                    self.final_success_printed = True
                return True
            return False

        elif self.stage_id > self.total_stages:
            # 防御性保护，正常情况下不应到达此处
            self.fail_flag = True
            return False

        # ==============================================================
        # 4. 阶段判定（0 ~ 11）
        # ==============================================================

        stage_type = self.stage_id % 6  # 0=A→B, 1=左复位, 2=铃铛, 3=右复位, 4=B→A, 5=左复位

        if stage_type == 0:
            # --- 阶段 A→B ---
            # 在方块已被抬起的前提下，检测方块落在 B 点
            if self._is_block_on_B() and self.block_lifted:
                flushed_print(
                    f"✓ 阶段 {self.stage_id} 通过：方块已放至 B 点 "
                    f"(进度: {self.stage_id + 1}/{self.total_stages})"
                )
                self.block_lifted = False  # 重置磁滞
                self.stage_id += 1
                
                # BUG 1 修复：强制要求进入下一个点击铃铛阶段时，必须先经历真实的物理松开动作
                self.bell_pressed_flag = False
                self.bell_lifted_flag = False
                
                self.max_reward = max(self.max_reward, self.stage_id / float(self.total_stages))
                return False

            # 异常检测：方块已被抬起，但落回了 A 点（放回原处）
            if self.block_lifted and self._is_block_on_A():
                flushed_print(
                    f"❌ 失败（阶段 {self.stage_id}）：方块被抬起后放回了 A 点，"
                    "但此阶段要求放到 B 点。"
                )
                self.fail_flag = True
                return False

        elif stage_type == 1:
            # --- 阶段：左臂复位 ---
            # 约束：左臂复位期间，方块必须安分地呆在 B 点
            if not self._is_block_on_B() or self.block_lifted:
                flushed_print(f"❌ 失败（阶段 {self.stage_id}）：左臂复位期间，方块被异常移动（必须停留在 B 点）。")
                self.fail_flag = True
                return False

            if self._is_left_arm_reset():
                flushed_print(f"✓ 阶段 {self.stage_id} 通过：左臂已复位 (进度: {self.stage_id + 1}/{self.total_stages})")
                self.stage_id += 1
                self.max_reward = max(self.max_reward, self.stage_id / float(self.total_stages))
                return False

        elif stage_type == 2:
            # --- 阶段：点击铃铛 ---
            # BUG 2 修复：在整个找铃铛和按铃铛的过程中，左臂绝不能乱动方块！
            # 方块必须安分地呆在 B 点。
            if not self._is_block_on_B() or self.block_lifted:
                flushed_print("❌ 失败：在点击铃铛阶段，方块必须始终停留在 B 点（严禁在此阶段移动方块）。")
                self.fail_flag = True
                return False

            # 每次刚进入铃铛阶段（从上一阶段推进过来）时，
            # 强制重置双磁滞标志，确保两次铃铛阶段相互独立。
            currently_touching = self._is_bell_touched()

            if not currently_touching:
                # 未接触铃铛时：重置磁滞，允许下次触发
                self.bell_pressed_flag = False
                self.bell_lifted_flag  = True
            else:
                # 正在接触铃铛
                if not self.bell_pressed_flag and self.bell_lifted_flag:
                    # 检测到新的一次有效接触（之前已确认离开过铃铛）
                    self.bell_pressed_flag = True
                    self.bell_lifted_flag  = False
                    flushed_print(
                        f"✓ 阶段 {self.stage_id} 通过：铃铛已被点击 "
                        f"(进度: {self.stage_id + 1}/{self.total_stages})"
                    )
                    self.stage_id += 1
                    self.max_reward = max(self.max_reward, self.stage_id / float(self.total_stages))
                    return False

        elif stage_type == 3:
            # --- 阶段：右臂复位 ---
            # 约束：右臂复位期间，方块仍需安分地呆在 B 点
            if not self._is_block_on_B() or self.block_lifted:
                flushed_print(f"❌ 失败（阶段 {self.stage_id}）：右臂复位期间，方块被异常移动（必须停留在 B 点）。")
                self.fail_flag = True
                return False

            if self._is_right_arm_reset():
                flushed_print(f"✓ 阶段 {self.stage_id} 通过：右臂已复位 (进度: {self.stage_id + 1}/{self.total_stages})")
                self.stage_id += 1
                self.max_reward = max(self.max_reward, self.stage_id / float(self.total_stages))
                return False

        elif stage_type == 4:
            # --- 阶段 B→A ---
            # 在方块已被抬起的前提下，检测方块落在 A 点
            if self._is_block_on_A() and self.block_lifted:
                flushed_print(
                    f"✓ 阶段 {self.stage_id} 通过：方块已放回 A 点 "
                    f"(进度: {self.stage_id + 1}/{self.total_stages})"
                )
                self.block_lifted = False  # 重置磁滞
                self.stage_id += 1
                self.max_reward = max(self.max_reward, self.stage_id / float(self.total_stages))
                return False

            # 异常检测：方块已被抬起，但落回了 B 点（拖拽 /放回错误位置）
            if self.block_lifted and self._is_block_on_B():
                flushed_print(
                    f"❌ 失败（阶段 {self.stage_id}）：方块被抬起后放回了 B 点，"
                    "但此阶段要求放回 A 点。"
                )
                self.fail_flag = True
                return False

        elif stage_type == 5:
            # --- 阶段：左臂复位 ---
            # 约束：左臂复位期间，方块必须停留在 A 点
            if not self._is_block_on_A() or self.block_lifted:
                flushed_print(f"❌ 失败（阶段 {self.stage_id}）：左臂复位期间，方块被异常移动（必须停留在 A 点）。")
                self.fail_flag = True
                return False

            if self._is_left_arm_reset():
                flushed_print(f"✓ 阶段 {self.stage_id} 通过：左臂已复位 (进度: {self.stage_id + 1}/{self.total_stages})")
                self.stage_id += 1
                self.max_reward = max(self.max_reward, self.stage_id / float(self.total_stages))
                return False

        return False


if __name__ == "__main__":
    pass
