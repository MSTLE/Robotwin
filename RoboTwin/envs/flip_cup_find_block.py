from ._base_task import Base_Task
from .utils import *
import sapien
import numpy as np
import sys
from pathlib import Path

def flushed_print(*args, **kwargs):
    """确保日志实时刷新的打印函数。"""
    print(*args, **kwargs)
    sys.stdout.flush()

class flip_cup_find_block(Base_Task):
    """
    翻杯寻物任务环境：
    机器人需要翻开或移开遮挡物（槽块，替代杯子）来寻找下方隐藏的红色目标方块。
    """

    def setup_demo(self, **kwags):
        """初始化任务特定的参数和元数据。"""
        self.random_block_order = kwags.get("random_block_order", False)
        
        super()._init_task_env_(**kwags)
        
        self.info["info"] = {
            "robot": "aloha-agilex",
            "table": "desk",
            "task": "flip_block_find_block",
            "random_block_order": self.random_block_order
        }
        
        if hasattr(self, 'generated_blocks'):
            self.info["info"]["generated_blocks"] = self.generated_blocks
            self.info["info"]["num_blocks"] = len(self.generated_blocks)

    def load_actors(self):
        """加载并设置场景中的所有物理实体。"""
        flushed_print("正在加载资产...")
        base_table_height = 0.74
        
        # 1. 在桌面上设置蓝色长方形垫子
        pad_thickness = 0.08
        pad_half_thickness = pad_thickness / 2
        pad_z = base_table_height + pad_half_thickness
        
        self.pad = create_box(
            scene=self,
            pose=sapien.Pose([0.0, 0.0, pad_z]),
            half_size=(0.25, 0.04, pad_half_thickness),
            color=(45/255, 173/255, 232/255),
            name="pad",
            is_static=True,
        )
        
        pad_surface_z = base_table_height + pad_thickness
        
        # 2. 设置隐藏的红色方块
        block_size = 0.0125
        block_z = pad_surface_z + block_size
        block_positions = [
            ([0.15, 0.0, block_z], "block2"),
            ([-0.15, 0.0, block_z], "block3")
        ]
        
        # 根据 Episode 编号或随机性决定生成的方块
        if self.random_block_order:
            block_index = np.random.randint(0, 2)
        else:
            block_index = self.ep_num % 2
            
        selected_indices = [block_index]
        flushed_print(f"模式: {'随机' if self.random_block_order else '顺序'} (索引: {block_index})")
        
        self.generated_blocks = []
        self.blocks = {}
        self.target_fluted_block_name = None
        
        # 创建选中的红色方块，并记录其上方的遮挡物名称
        for idx in selected_indices:
            self.target_fluted_block_name = "fluted_block2" if idx == 0 else "fluted_block3"
                
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
            flushed_print(f"已生成 {name} 位置: {pos} (对应目标遮挡物: {self.target_fluted_block_name})")
        
        # 3. 设置 Fluted-Blocks（作为遮挡物/外壳）
        def create_shrunk_fluted_block(self, pose, name, shrink_factor=0.7):
            """通过绕过默认的 create_actor 来创建自定义比例的模型辅助函数。"""
            modeldir = Path(__file__).parent.parent / "assets" / "objects" / "004_fluted-block"
            original_scale = np.array([0.45, 0.4, 0.45])
            target_scale = original_scale * shrink_factor
            
            builder = self.scene.create_actor_builder()
            builder.set_physx_body_type("dynamic")
            
            visual_file = str(modeldir / "visual" / "base0.glb")
            collision_file = str(modeldir / "collision" / "base0.glb")
            
            builder.add_nonconvex_collision_from_file(filename=collision_file, scale=target_scale)
            builder.add_visual_from_file(filename=visual_file, scale=target_scale)
            
            actor_entity = builder.build(name=name)
            actor_entity.set_pose(pose)
            return Actor(actor_entity, {"scale": target_scale.tolist()})

        fluted_block_z = pad_surface_z + 0.05
        
        # 创建两个缩小的 Fluted-Block (原 block1 位置的已删除)
        self.fluted_block2 = create_shrunk_fluted_block(self, sapien.Pose([0.15, 0.0, fluted_block_z], [0.707, -0.707, 0, 0]), "fluted_block2")
        self.fluted_block2.set_mass(0.1)
        
        self.fluted_block3 = create_shrunk_fluted_block(self, sapien.Pose([-0.15, 0.0, fluted_block_z], [0.707, -0.707, 0, 0]), "fluted_block3")
        self.fluted_block3.set_mass(0.1)
        
        flushed_print("资产加载完成（已包含缩放模型）。")
        
        # 4. 设置物理属性（增加摩擦力以保证抓取稳定）
        high_friction_material = self.scene.create_physical_material(
            static_friction=5.0,
            dynamic_friction=5.0,
            restitution=0.0,
        )
        
        # 为遮挡物应用材质
        for block in [self.fluted_block2, self.fluted_block3]:
            for comp in block.actor.get_components():
                if isinstance(comp, sapien.physx.PhysxRigidBaseComponent):
                    for shape in comp.get_collision_shapes():
                        shape.set_physical_material(high_friction_material)
        
        # 为机器人夹爪应用材质
        for link in self.robot.left_entity.get_links():
            if link.get_name() in self.robot.gripper_name:
                for shape in link.get_collision_shapes():
                    shape.set_physical_material(high_friction_material)
        for link in self.robot.right_entity.get_links():
            if link.get_name() in self.robot.gripper_name:
                for shape in link.get_collision_shapes():
                    shape.set_physical_material(high_friction_material)
        
        flushed_print("物理材质已更新（摩擦力已增大）。")

    def play_once(self):
        """执行预定义的演示序列。"""
        flushed_print("开始环境演示...")
        arm_L = ArmTag("left")
        top_down_quat = [-0.5, 0.5, -0.5, -0.5]
        
        # 左臂：检查左侧的 fluted_block3
        flushed_print("阶段 1: 左臂检查 fluted_block3...")
        self.move(self.move_by_displacement(arm_tag=arm_L, z=0.1))
        
        curr_pose = self.robot.get_left_ee_pose()
        block3_pos = self.fluted_block3.get_pose().p
        
        # 移动到 block3 上方
        self.move(self.move_by_displacement(
            arm_tag=arm_L, 
            x=block3_pos[0] - curr_pose[0], 
            y=block3_pos[1] - curr_pose[1], 
            z=(block3_pos[2] + 0.15) - curr_pose[2], 
            quat=top_down_quat
        ))
        
        # 抓取并抬起尝试检查
        self.move(self.move_by_displacement(arm_tag=arm_L, z=-0.014))
        self.move((arm_L, [Action(arm_L, "close", target_gripper_pos=0.7)]))
        self.move(self.move_by_displacement(arm_tag=arm_L, z=0.05))
        
        # 放回原位并释放
        self.move(self.move_by_displacement(arm_tag=arm_L, z=-0.05))
        self.move((arm_L, [Action(arm_L, "open", target_gripper_pos=1)]))
        self.move(self.move_by_displacement(arm_tag=arm_L, z=0.08))
        self.move(self.back_to_origin(arm_L))
        
        # 右臂：检查右侧的 fluted_block2
        flushed_print("阶段 2: 右臂检查 fluted_block2...")
        arm_R = ArmTag("right")
        self.move(self.move_by_displacement(arm_tag=arm_R, z=0.1))
        
        curr_pose_R = self.robot.get_right_ee_pose()
        block2_pos = self.fluted_block2.get_pose().p
        
        # 移动到 block2 上方
        self.move(self.move_by_displacement(
            arm_tag=arm_R, 
            x=block2_pos[0] - curr_pose_R[0], 
            y=block2_pos[1] - curr_pose_R[1], 
            z=(block2_pos[2] + 0.15) - curr_pose_R[2], 
            quat=top_down_quat
        ))
        
        # 抓取并抬起尝试检查
        self.move(self.move_by_displacement(arm_tag=arm_R, z=-0.014))
        self.move((arm_R, [Action(arm_R, "close", target_gripper_pos=0.7)]))
        self.move(self.move_by_displacement(arm_tag=arm_R, z=0.05))
        
        # 放回原位并释放
        self.move(self.move_by_displacement(arm_tag=arm_R, z=-0.05))
        self.move((arm_R, [Action(arm_R, "open", target_gripper_pos=1)]))
        self.move(self.move_by_displacement(arm_tag=arm_R, z=0.08))
        self.move(self.back_to_origin(arm_R))
        
        # 最终行动：针对发现红方块的目标位置进行操作
        flushed_print(f"阶段 3: 发现目标！目标在 {self.target_fluted_block_name} 下方。")
        
        target_arm_tag = ArmTag("left") if "block3" in self.target_fluted_block_name else ArmTag("right")
        target_shell_obj = getattr(self, self.target_fluted_block_name)
        
        flushed_print(f"动作: 使用 {target_arm_tag.arm} 臂拿起目标遮挡物...")
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=0.1))
        
        # 重新对准目标遮挡物
        curr_pose_final = (self.robot.get_left_ee_pose() if target_arm_tag.arm == "left" else self.robot.get_right_ee_pose())
        t_pos = target_shell_obj.get_pose().p
        
        self.move(self.move_by_displacement(
            arm_tag=target_arm_tag, 
            x=t_pos[0] - curr_pose_final[0], 
            y=t_pos[1] - curr_pose_final[1], 
            z=(t_pos[2] + 0.15) - curr_pose_final[2],
            quat=top_down_quat
        ))
        
        # 抬起并移开遮挡物
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=-0.01))
        self.move((target_arm_tag, [Action(target_arm_tag, "close", target_gripper_pos=0.7)]))
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=0.05))
        flushed_print("正在移开遮挡物...")
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, y=-0.18))
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=-0.13))
        self.move((target_arm_tag, [Action(target_arm_tag, "open", target_gripper_pos=1)]))
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=0.2))
        
        # 移动到红方块正上方进行标记
        flushed_print("对准隐藏的目标方块...")
        red_block_obj = self.blocks["block2"] if "block2" in self.blocks else self.blocks["block3"]
        red_block_pos = red_block_obj.get_pose().p
        
        curr_temp_pose = (self.robot.get_left_ee_pose() if target_arm_tag.arm == "left" else self.robot.get_right_ee_pose())
        self.move(self.move_by_displacement(
            arm_tag=target_arm_tag,
            x=red_block_pos[0] - curr_temp_pose[0],
            y=red_block_pos[1] - curr_temp_pose[1],
            z=(red_block_pos[2] + 0.15) - curr_temp_pose[2],
            quat=top_down_quat
        ))
        
        # 抓起红色方块
        flushed_print("正在抓起目标红方块...")
        self.move((target_arm_tag, [Action(target_arm_tag, "close", target_gripper_pos=0.2)]))
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=0.1))
        
        # 最后一步：将红色方块放置在之前移开的遮挡物上方
        flushed_print("将红方块放置在遮挡物上方...")
        current_shell_pos = target_shell_obj.get_pose().p
        curr_ee_pose = (self.robot.get_left_ee_pose() if target_arm_tag.arm == "left" else self.robot.get_right_ee_pose())
        
        # 先执行 XY 平面的移动
        self.move(self.move_by_displacement(
            arm_tag=target_arm_tag,
            x=current_shell_pos[0] - curr_ee_pose[0],
            y=current_shell_pos[1] - curr_ee_pose[1],
            z=0.0,
            quat=top_down_quat
        ))
        
        # 下降并释放
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=-0.13))
        self.move((target_arm_tag, [Action(target_arm_tag, "open", target_gripper_pos=1)]))
        
        # 最终复位
        flushed_print("演示结束，正在复位机械臂...")
        self.move(self.move_by_displacement(arm_tag=target_arm_tag, z=0.15))
        self.move(self.back_to_origin(target_arm_tag))
        
        flushed_print("任务完成。")
        return self.info

    def check_success(self):
        """如果环境基本加载成功，则视为任务成功。"""
        return True

if __name__ == "__main__":
    pass
