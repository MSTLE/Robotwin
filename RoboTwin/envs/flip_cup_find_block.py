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
        super()._init_task_env_(**kwags)
        # 填充 episode 信息
        self.info["info"] = {
            "robot": "aloha-agilex",
            "table": "desk",
            "task": "flip_cup_find_block"
        }

    def load_actors(self):
        flushed_print("正在加载资产...")
        # Base_Task._init_task_env_ 已经调用了 create_table_and_wall
        
        # 在桌面上创建三个物块
        # 物块1: 原点位置
        self.block1 = create_box(
            scene=self,
            pose=sapien.Pose([0.0, 0.0, 0.77]),
            half_size=(0.02, 0.02, 0.02),
            color=(1, 0, 0),
            name="block1",
            is_static=False,
        )
        self.block1.set_mass(0.05)
        
        # 物块2: 原点 x+0.2
        self.block2 = create_box(
            scene=self,
            pose=sapien.Pose([0.2, 0.0, 0.77]),
            half_size=(0.02, 0.02, 0.02),
            color=(1, 0, 0),
            name="block2",
            is_static=False,
        )
        self.block2.set_mass(0.05)
        
        # 物块3: 原点 x-0.2
        self.block3 = create_box(
            scene=self,
            pose=sapien.Pose([-0.2, 0.0, 0.77]),
            half_size=(0.02, 0.02, 0.02),
            color=(1, 0, 0),
            name="block3",
            is_static=False,
        )
        self.block3.set_mass(0.05)
        
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
