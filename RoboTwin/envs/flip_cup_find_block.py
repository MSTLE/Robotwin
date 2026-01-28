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
        
        # 定义三个物块的位置
        block_positions = [
            ([0.0, 0.0, 0.77], "block1"),
            ([0.2, 0.0, 0.77], "block2"),
            ([-0.2, 0.0, 0.77], "block3")
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
        
        # 在桌面上创建选中的物块
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
        
        # 在三个物块上方创建三个杯子 (使用 021_cup, model_id=3)
        # 杯子需要放在物块上方,z坐标需要加上物块高度(0.04)和杯子底部到中心的距离
        
        # 杯子1: 在物块1上方
        self.cup1 = create_actor(
            scene=self,
            pose=sapien.Pose([0.0, 0.0, 0.82], [0.707, -0.707, 0, 0]),  # 桌面0.74 + 杯子中心偏移0.08
            modelname="021_cup",
            model_id=3,
            convex=False,  # 非凸包,允许夹爪伸进杯子内部
            is_static=False,  # 动态物体,可以被机器人夹起
        )
        self.cup1.set_name("cup1")
        self.cup1.set_mass(0.1)  # 设置杯子质量为100克
        
        # 杯子2: 在物块2上方
        self.cup2 = create_actor(
            scene=self,
            pose=sapien.Pose([0.2, 0.0, 0.82], [0.707, -0.707, 0, 0]),  # 桌面0.74 + 杯子中心偏移0.08
            modelname="021_cup",
            model_id=3,
            convex=False,  # 非凸包,允许夹爪伸进杯子内部
            is_static=False,  # 动态物体,可以被机器人夹起
        )
        self.cup2.set_name("cup2")
        self.cup2.set_mass(0.1)  # 设置杯子质量为100克
        
        # 杯子3: 在物块3上方
        self.cup3 = create_actor(
            scene=self,
            pose=sapien.Pose([-0.2, 0.0, 0.82], [0.707, -0.707, 0, 0]),  # 桌面0.74 + 杯子中心偏离0.08
            modelname="021_cup",
            model_id=3,
            convex=False,  # 非凸包,允许夹爪伸进杯子内部
            is_static=False,  # 动态物体,可以被机器人夹起
        )
        self.cup3.set_name("cup3")
        self.cup3.set_mass(0.1)  # 设置杯子质量为100克
        
        flushed_print("资产加载完成（机器人、桌子、三个物块和三个杯子）。")

    def play_once(self):
        flushed_print("执行基本的环境演示...")
        # 仅执行一个简单的动作以验证环境运行正常
        arm_L = ArmTag("left")
        self.move(self.back_to_origin(arm_L))
        flushed_print("环境演示完成。")
        return self.info

    def check_success(self):
        # 基本环境加载成功即视为 True
        return True

if __name__ == "__main__":
    pass
