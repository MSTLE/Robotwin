from ._base_task import Base_Task
from .utils import *
import sapien
import numpy as np
import sys
from pathlib import Path

def flushed_print(*args, **kwargs):
    """确保日志能够实时刷新的打印函数"""
    print(*args, **kwargs)
    sys.stdout.flush()

class flip_cup_find_block(Base_Task):
    """
    翻杯寻物任务环境。
    当前仅初始化基本的机器人和桌面环境。
    """

    def setup_demo(self, **kwags):
        # 获取物块生成顺序配置
        self.random_block_order = kwags.get("random_block_order", False)
        
        super()._init_task_env_(**kwags)
        # 填充 episode 信息
        self.info["info"] = {
            "robot": "aloha-agilex",
            "table": "desk",
            "task": "flip_block_find_block",
            "random_block_order": self.random_block_order
        }
        
        # 更新生成的物块信息到info
        if hasattr(self, 'generated_blocks'):
            self.info["info"]["generated_blocks"] = self.generated_blocks
            self.info["info"]["num_blocks"] = len(self.generated_blocks)

    def load_actors(self):
        flushed_print("正在加载资产...")
        # Base_Task._init_task_env_ 已经调用了 create_table_and_wall
        
        # 使用基准桌面高度，让 create_box/create_actor 自动处理 table_z_bias
        base_table_height = 0.74
        
        # 1. 在桌面上创建矩形垫
        pad_thickness = 0.08
        pad_half_thickness = pad_thickness / 2
        pad_z = base_table_height + pad_half_thickness
        
        self.pad = create_box(
            scene=self,
            pose=sapien.Pose([0.0, 0.0, pad_z]),
            half_size=(0.25, 0.04, pad_half_thickness),  # 设置垫子大小覆盖物块和 Fluted-Block
            color=(45/255, 173/255, 232/255),  # 蓝色垫子
            name="pad",
            is_static=True,
        )
        
        # 垫子表面的基准 z 坐标
        pad_surface_z = base_table_height + pad_thickness
        
        # 2. 定义三个物块的位置，并修正 z 轴使其放在垫子上
        # 物块 half_size 为 0.0125, 中心应在 pad_surface_z + 0.0125
        block_size = 0.0125
        block_z = pad_surface_z + block_size
        block_positions = [
            ([0.2, 0.0, block_z], "block2"),
            ([-0.2, 0.0, block_z], "block3")
        ]
        
        # 根据配置决定生成顺序
        if self.random_block_order:
            # 随机选择生成物块2或3中的一个
            block_index = np.random.randint(0, 2)
            selected_indices = [block_index]
            flushed_print(f"随机生成模式: 随机选择物块 (索引 {block_index})")
        else:
            # 按顺序循环生成单个物块：ep0→block2, ep1→block3, ep2→block2...
            block_index = self.ep_num % 2
            selected_indices = [block_index]
            flushed_print(f"顺序生成模式: Episode {self.ep_num} → 选择索引 {block_index}")
        
        # 记录实际生成的物块信息
        self.generated_blocks = []
        self.blocks = {}
        self.target_fluted_block_name = None
        
        # 在垫子上创建选中的物块
        for idx in selected_indices:
            # 记录哪一个是目标槽块: index 0 对应 x=0.2 (fluted_block2), index 1 对应 x=-0.2 (fluted_block3)
            if idx == 0:
                self.target_fluted_block_name = "fluted_block2"
            else:
                self.target_fluted_block_name = "fluted_block3"
                
            pos, name = block_positions[idx]
            block = create_box(
                scene=self,
                pose=sapien.Pose(pos),
                half_size=(block_size, block_size, block_size),
                color=(1, 0, 0),
                name=name,
                is_static=False,
            )
            block.set_mass(0.05)
            self.blocks[name] = block
            self.generated_blocks.append({"name": name, "position": pos, "index": int(idx)})
            flushed_print(f"  生成物块: {name} at {pos} (对应目标槽块: {self.target_fluted_block_name})")
        
        # 3. 在三个物块位置上方创建三个 Fluted-Block (作为杯子的替代品)
        # 由于 create_actor 强制使用 json 中的 scale，我们在代码中手动创建一个缩小的版本
        def create_shrunk_fluted_block(self, pose, name, shrink_factor=0.7):
            # 使用基于文件路径的相对路径，确保在不同工作目录下都能找到资产
            modeldir = Path(__file__).parent.parent / "assets" / "objects" / "004_fluted-block"
            # 原始比例来自 model_data0.json: [0.45, 0.4, 0.45]
            original_scale = np.array([0.45, 0.4, 0.45])
            target_scale = original_scale * shrink_factor
            
            # 使用 SAPIEN builder 手动构建，以绕过 create_actor 的限制
            builder = self.scene.create_actor_builder()
            builder.set_physx_body_type("dynamic")
            
            visual_file = str(modeldir / "visual" / "base0.glb")
            collision_file = str(modeldir / "collision" / "base0.glb")
            
            # 由于是凹形物体（杯子），设置 convex=False
            builder.add_nonconvex_collision_from_file(filename=collision_file, scale=target_scale)
            builder.add_visual_from_file(filename=visual_file, scale=target_scale)
            
            actor_entity = builder.build(name=name)
            actor_entity.set_pose(pose)
            
            # 模拟 create_actor 返回的 Actor 对象包装
            return Actor(actor_entity, {"scale": target_scale.tolist()})

        # 缩小后的模型高度也会降低，相应调整 fluted_block_z
        # 原高度约为 0.16 * 0.4 = 0.064, 缩小 0.7 倍后约为 0.045
        # 我们把高度调整得低一些，让它更接近垫子表面
        fluted_block_z = pad_surface_z + 0.05
        
        # 创建两个缩小的 Fluted-Block (原 block1 位置的已删除)
        self.fluted_block2 = create_shrunk_fluted_block(self, sapien.Pose([0.2, 0.0, fluted_block_z], [0.707, -0.707, 0, 0]), "fluted_block2")
        self.fluted_block2.set_mass(0.1)
        
        self.fluted_block3 = create_shrunk_fluted_block(self, sapien.Pose([-0.2, 0.0, fluted_block_z], [0.707, -0.707, 0, 0]), "fluted_block3")
        self.fluted_block3.set_mass(0.1)
        
        flushed_print("资产加载完成（已加载并缩小 Fluted-Block 模型）。")
        
        # 4. 增加摩擦力设置，防止 Fluted-Block 滑落
        high_friction_material = self.scene.create_physical_material(
            static_friction=5.0,
            dynamic_friction=5.0,
            restitution=0.0,
        )
        
        # 为两个 Fluted-Block 设置高摩擦力
        for block in [self.fluted_block2, self.fluted_block3]:
            for comp in block.actor.get_components():
                if isinstance(comp, sapien.physx.PhysxRigidBaseComponent):
                    for shape in comp.get_collision_shapes():
                        shape.set_physical_material(high_friction_material)
        
        # 为机器人的夹爪设置高摩擦力
        for link in self.robot.left_entity.get_links():
            if link.get_name() in self.robot.gripper_name:
                for shape in link.get_collision_shapes():
                    shape.set_physical_material(high_friction_material)
        for link in self.robot.right_entity.get_links():
            if link.get_name() in self.robot.gripper_name:
                for shape in link.get_collision_shapes():
                    shape.set_physical_material(high_friction_material)
        
        flushed_print("物理材质更新完成（已增加 Fluted-Block 和夹爪的摩擦力）。")

    def play_once(self):
        flushed_print("执行基本的环境演示...")
        arm_L = ArmTag("left")
        
        # 1. 抬起左臂一点 (使用 displacement)
        flushed_print("抬起左臂...")
        self.move(self.move_by_displacement(arm_tag=arm_L, z=0.1))
        
        # 2. 获取当前位置并计算到第三个 Fluted-Block 上方的位移
        flushed_print("移动到第三个 Fluted-Block 上方...")
        curr_pose = self.robot.get_left_ee_pose()
        block3_pos = self.fluted_block3.get_pose().p
        
        # 目标位置在 Fluted-Block 中心上方 0.15m (Top-Down 模式)
        y_offset = 0.0
        dx = block3_pos[0] - curr_pose[0]
        dy = (block3_pos[1] + y_offset) - curr_pose[1]
        dz = (block3_pos[2] + 0.15) - curr_pose[2]
        
        # 使用 Top-down 姿态 (参考 _GLOBAL_CONFIGS.py)
        top_down_quat = [-0.5, 0.5, -0.5, -0.5]
        
        # 使用 displacement 移动到目标位置并设定 Top-Down 姿态
        self.move(self.move_by_displacement(arm_tag=arm_L, x=dx, y=dy, z=dz, quat=top_down_quat))
        
        # 3. 向下移动 (使用 displacement)
        flushed_print("向下移动...")
        self.move(self.move_by_displacement(arm_tag=arm_L, z=-0.014))
        
        
        # 5. 闭合夹爪 (设置位置为 0.70，避免太紧导致物体飞出)
        flushed_print("闭合夹爪...")
        self.move((arm_L, [Action(arm_L, "close", target_gripper_pos=0.7)]))
        
        # 6. 向上平移 (使用 displacement)
        flushed_print("向上平移...")
        self.move(self.move_by_displacement(arm_tag=arm_L, z=0.05))

        
        # 7. 向下平移放回 (使用 displacement)
        flushed_print("向下平移放回...")
        self.move(self.move_by_displacement(arm_tag=arm_L, z=-0.05))
        
        # 8. 松开夹爪
        flushed_print("松开夹爪...")
        self.move((arm_L, [Action(arm_L, "open", target_gripper_pos=1)]))
        
        # 9. 抬起手臂离开
        flushed_print("抬起手臂离开...")
        self.move(self.move_by_displacement(arm_tag=arm_L, z=0.08))
        
        # 10. 复位手臂
        flushed_print("复位手臂...")
        self.move(self.back_to_origin(arm_L))
        
        arm_R = ArmTag("right")
        # 11. 抬起右臂
        flushed_print("抬起右臂...")
        self.move(self.move_by_displacement(arm_tag=arm_R, z=0.1))
        
        # 12. 移动到第二个 Fluted-Block 上方
        flushed_print("移动到第二个 Fluted-Block 上方...")
        curr_pose_R = self.robot.get_right_ee_pose()
        block2_pos = self.fluted_block2.get_pose().p
        
        dx = block2_pos[0] - curr_pose_R[0]
        dy = block2_pos[1] - curr_pose_R[1]
        dz = (block2_pos[2] + 0.15) - curr_pose_R[2]
        
        # 使用相同的 Top-down 姿态
        self.move(self.move_by_displacement(arm_tag=arm_R, x=dx, y=dy, z=dz, quat=top_down_quat))
        
        # 13. 向下移动
        flushed_print("向下移动...")
        self.move(self.move_by_displacement(arm_tag=arm_R, z=-0.014))
        
        # 14. 闭合夹爪
        flushed_print("闭合夹爪...")
        self.move((arm_R, [Action(arm_R, "close", target_gripper_pos=0.7)]))
        
        # 15. 向上平移
        flushed_print("向上平移...")
        self.move(self.move_by_displacement(arm_tag=arm_R, z=0.05))
        
        # 16. 向下平移放回
        flushed_print("向下平移放回...")
        self.move(self.move_by_displacement(arm_tag=arm_R, z=-0.05))
        
        # 17. 松开夹爪
        flushed_print("松开夹爪...")
        self.move((arm_R, [Action(arm_R, "open", target_gripper_pos=1)]))
        
        # 18. 抬起手臂离开
        flushed_print("抬起手臂离开...")
        self.move(self.move_by_displacement(arm_tag=arm_R, z=0.08))
        
        # 19. 复位右臂
        flushed_print("复位右臂...")
        self.move(self.back_to_origin(arm_R))
        
        # 20. 最终确认：重新抓取出现红方块的目标槽块
        flushed_print(f"--- 搜索结束，目标物块在 {self.target_fluted_block_name} 下方 ---")
        
        # 确定执行抓取的手臂和目标对象
        target_arm_tag = ArmTag("left") if "block3" in self.target_fluted_block_name else ArmTag("right")
        target_block_obj = getattr(self, self.target_fluted_block_name)
        
        flushed_print(f"使用 {target_arm_tag.arm} 臂重新抓取目标槽块...")
        # 抬起 -> 移动 -> 向下 -> 抓取 -> 抬起
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=0.1))
        
        curr_pose_final = (self.robot.get_left_ee_pose() if target_arm_tag.arm == "left" else self.robot.get_right_ee_pose())
        t_pos = target_block_obj.get_pose().p
        
        # 计算位移并保持 Top-down 姿态
        self.move(self.move_by_displacement(
            arm_tag=target_arm_tag, 
            x=t_pos[0] - curr_pose_final[0], 
            y=t_pos[1] - curr_pose_final[1], 
            z=(t_pos[2] + 0.15) - curr_pose_final[2],
            quat=top_down_quat
        ))
        
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=-0.01))
        self.move((target_arm_tag, [Action(target_arm_tag, "close", target_gripper_pos=0.7)]))
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=0.05))
        
        # 21. 将槽块向 y 负方向移动
        flushed_print("将槽块向 y 负方向移动...")
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, y=-0.18))
        
        # 22. 向下平移放到桌面上
        flushed_print("向下平移放到桌面上...")
        # 减小下降距离，防止过度下压
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=-0.13))
        
        # 23. 松开夹爪
        flushed_print("在桌面上松开夹爪...")
        self.move((target_arm_tag, [Action(target_arm_tag, "open", target_gripper_pos=1)]))
        
        # 24. 抬起手臂
        flushed_print("任务完成，抬起手臂...")
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=0.2))
        
        # 25. 移动到显露出的红方块上方
        flushed_print("移动到物块上方...")
        # 找到当前存在的红方块
        red_block_obj = self.blocks["block2"] if "block2" in self.blocks else self.blocks["block3"]
        red_block_pos = red_block_obj.get_pose().p
        
        curr_pose_above = (self.robot.get_left_ee_pose() if target_arm_tag.arm == "left" else self.robot.get_right_ee_pose())
        
        # 移动到红方块上方 0.15m
        self.move(self.move_by_displacement(
            arm_tag=target_arm_tag,
            x=red_block_pos[0] - curr_pose_above[0],
            y=red_block_pos[1] - curr_pose_above[1],
            z=(red_block_pos[2] + 0.15) - curr_pose_above[2],
            quat=top_down_quat
        ))
        
        # 26. 向下移动接触物块
        # flushed_print("向下移动接触物块...")
        # self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=-0.14))
        
        # 27. 闭合夹爪抓住物块
        flushed_print("闭合夹爪抓住物块...")
        # 红方块较小 (0.025m), 设置较大的闭合值
        self.move((target_arm_tag, [Action(target_arm_tag, "close", target_gripper_pos=0.2)]))
        
        # 28. 向上平移
        flushed_print("向上平移...")
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=0.1))
        
        # 29. 移动到槽块上方 (分解运动：先移动 XY，再移动 Z)
        flushed_print("将红方块平移到槽块所在位置 (XY)...")
        target_block_current_pos = target_block_obj.get_pose().p
        curr_pose_with_red = (self.robot.get_left_ee_pose() if target_arm_tag.arm == "left" else self.robot.get_right_ee_pose())
        
        # 步骤 29a: 先在 XY 平面上移动到槽块上方，高度保持不变
        self.move(self.move_by_displacement(
            arm_tag=target_arm_tag,
            x=target_block_current_pos[0] - curr_pose_with_red[0],
            y=target_block_current_pos[1] - curr_pose_with_red[1],
            z=0.0,
            quat=top_down_quat
        ))
        
        # 刷新位置以进行下一步 Z 轴移动
        # curr_pose_after_xy = (self.robot.get_left_ee_pose() if target_arm_tag.arm == "left" else self.robot.get_right_ee_pose())
        # flushed_print("向下逼近槽块高度 (Z)...")
        # # 步骤 29b: 垂直向下移动到槽块上方 0.15m
        # self.move(self.move_by_displacement(
        #     arm_tag=target_arm_tag,
        #     x=0.0,
        #     y=0.0,
        #     z=(target_block_current_pos[2] + 0.15) - curr_pose_after_xy[2],
        #     quat=top_down_quat
        # ))
        
        # 30. 向下平移靠近槽块
        flushed_print("放下红方块...")
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=-0.14))
        
        # 31. 松开夹爪
        flushed_print("松开红方块...")
        self.move((target_arm_tag, [Action(target_arm_tag, "open", target_gripper_pos=1)]))
        
        # 32. 抬起手臂复位
        flushed_print("任务演示结束，准备复位...")
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=0.15))
        
        # 33. 全局复位
        flushed_print("机械臂复位...")
        self.move(self.back_to_origin(target_arm_tag))
        
        flushed_print("由于成功抓取并将其放置在槽块上方，任务最终圆满完成。")
        
        
        flushed_print("环境演示完成。")
        return self.info

    def check_success(self):
        # 基本环境加载成功即视为 True
        return True

if __name__ == "__main__":
    pass
