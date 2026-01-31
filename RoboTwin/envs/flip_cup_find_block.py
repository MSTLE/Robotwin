from ._base_task import Base_Task
from .utils import *
import sapien
import numpy as np
import sys

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
            "task": "flip_cup_find_block",
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
            half_size=(0.25, 0.045, pad_half_thickness),  # 设置垫子大小覆盖物块和杯子
            color=(45/255, 173/255, 232/255),  # 蓝色垫子
            name="pad",
            is_static=True,
        )
        
        # 垫子表面的基准 z 坐标
        pad_surface_z = base_table_height + pad_thickness
        
        # 2. 定义三个物块的位置，并修正 z 轴使其放在垫子上
        # 物块 half_size 为 0.02, 中心应在 pad_surface_z + 0.02
        block_z = pad_surface_z + 0.02
        block_positions = [
            ([0.0, 0.0, block_z], "block1"),
            ([0.2, 0.0, block_z], "block2"),
            ([-0.2, 0.0, block_z], "block3")
        ]
        
        # 根据配置决定生成顺序
        if self.random_block_order:
            # 随机选择生成物块1、2或3中的一个
            block_index = np.random.randint(0, 3)
            selected_indices = [block_index]
            flushed_print(f"随机生成模式: 随机选择物块 {block_index + 1} (索引 {block_index})")
        else:
            # 按顺序循环生成单个物块：ep0→block1, ep1→block2, ep2→block3, ep3→block1...
            block_index = self.ep_num % 3
            selected_indices = [block_index]
            flushed_print(f"顺序生成模式: Episode {self.ep_num} → 生成物块 {block_index + 1} (索引 {block_index})")
        
        # 记录实际生成的物块信息
        self.generated_blocks = []
        self.blocks = {}
        
        # 在垫子上创建选中的物块
        for idx in selected_indices:
            pos, name = block_positions[idx]
            block = create_box(
                scene=self,
                pose=sapien.Pose(pos),
                half_size=(0.02, 0.02, 0.02),
                color=(1, 0, 0),
                name=name,
                is_static=False,
            )
            block.set_mass(0.05)
            self.blocks[name] = block
            self.generated_blocks.append({"name": name, "position": pos, "index": int(idx)})
            flushed_print(f"  生成物块: {name} at {pos}")
        
        # 3. 在三个物块位置上方创建三个杯子，并修正 z 轴使其放在垫子上
        # 杯子中心偏移量为 0.08, 中心应在 pad_surface_z + 0.08
        cup_z = pad_surface_z + 0.08
        
        # 杯子1: 在位置1
        self.cup1 = create_actor(
            scene=self,
            pose=sapien.Pose([0.0, 0.0, cup_z], [0.707, -0.707, 0, 0]),
            modelname="021_cup",
            model_id=3,
            convex=False,  # 非凸包,允许夹爪伸进杯子内部
            is_static=False,  # 动态物体,可以被机器人夹起
        )
        self.cup1.set_name("cup1")
        self.cup1.set_mass(0.1)  # 设置杯子质量为100克
        
        # 杯子2: 在位置2
        self.cup2 = create_actor(
            scene=self,
            pose=sapien.Pose([0.2, 0.0, cup_z], [0.707, -0.707, 0, 0]),
            modelname="021_cup",
            model_id=3,
            convex=False,  # 非凸包,允许夹爪伸进杯子内部
            is_static=False,  # 动态物体,可以被机器人夹起
        )
        self.cup2.set_name("cup2")
        self.cup2.set_mass(0.1)  # 设置杯子质量为100克
        
        # 杯子3: 在位置3
        self.cup3 = create_actor(
            scene=self,
            pose=sapien.Pose([-0.2, 0.0, cup_z], [0.707, -0.707, 0, 0]),
            modelname="021_cup",
            model_id=3,
            convex=False,  # 非凸包,允许夹爪伸进杯子内部
            is_static=False,  # 动态物体,可以被机器人夹起
        )
        self.cup3.set_name("cup3")
        self.cup3.set_mass(0.1)  # 设置杯子质量为100克
        
        flushed_print("资产加载完成（机器人、桌子、垫子、三个物块位置和三个杯子）。")
        
        # 4. 增加摩擦力设置，防止杯子滑落
        high_friction_material = self.scene.create_physical_material(
            static_friction=5.0,
            dynamic_friction=5.0,
            restitution=0.0,
        )
        
        # 为三个杯子设置高摩擦力
        for cup in [self.cup1, self.cup2, self.cup3]:
            for comp in cup.actor.get_components():
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
        
        flushed_print("物理材质更新完成（已增加杯子和夹爪的摩擦力）。")

    def play_once(self):
        flushed_print("执行基本的环境演示...")
        arm_L = ArmTag("left")
        
        # 1. 抬起左臂一点 (使用 displacement)
        flushed_print("抬起左臂...")
        self.move(self.move_by_displacement(arm_tag=arm_L, z=0.1))
        
        # 2. 获取当前位置并计算到第三个杯子上方的位移
        flushed_print("移动到第三个杯子上方...")
        curr_pose = self.robot.get_left_ee_pose()
        cup3_pos = self.cup3.get_pose().p
        
        # 目标位置在杯子中心上方 0.15m，并增加一个 Y 轴偏移
        y_offset = -0.15  # 向操作者方向偏移一点
        dx = cup3_pos[0] - curr_pose[0]
        dy = (cup3_pos[1] + y_offset) - curr_pose[1]
        dz = (cup3_pos[2] + 0.15) - curr_pose[2]
        
        # 使用 displacement 移动到目标位置
        self.move(self.move_by_displacement(arm_tag=arm_L, x=dx, y=dy, z=dz))
        
        # 3. 向下移动 (使用 displacement)
        flushed_print("向下移动...")
        self.move(self.move_by_displacement(arm_tag=arm_L, z=-0.17))
        
        # 4. 向 Y 正方向移动一点点 (使用 displacement)
        flushed_print("向 Y 正方向移动...")
        self.move(self.move_by_displacement(arm_tag=arm_L, y=0.025))
        
        # 5. 闭合夹爪 (设置位置为 0.8，避免太紧导致杯子飞出)
        flushed_print("闭合夹爪...")
        self.move((arm_L, [Action(arm_L, "close", target_gripper_pos=0.7)]))
        
        # 6. 向上平移 (使用 displacement)
        flushed_print("向上平移...")
        self.move(self.move_by_displacement(arm_tag=arm_L, z=0.15))
        
        # 6.5 对准中心 (由于之前抓取时有 Y 偏移，放回时需要补偿回来以对准物块)
        flushed_print("对准放回中心...")
        self.move(self.move_by_displacement(arm_tag=arm_L, y=0.025))
        
        # 7. 向下平移放回 (使用 displacement)
        flushed_print("向下平移放回...")
        self.move(self.move_by_displacement(arm_tag=arm_L, z=-0.15))
        
        # 8. 松开夹爪
        flushed_print("松开夹爪...")
        self.move((arm_L, [Action(arm_L, "open")]))
        
        # 9. 抬起手臂离开
        flushed_print("抬起手臂离开...")
        self.move(self.move_by_displacement(arm_tag=arm_L, z=0.2))
        
        flushed_print("环境演示完成。")
        return self.info

    def check_success(self):
        # 基本环境加载成功即视为 True
        return True

if __name__ == "__main__":
    pass
