
import sys
import os
import yaml
import numpy as np

# Add project root to path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.append(project_root)
os.chdir(project_root)

from envs.catch_bottle_and_shake import catch_bottle_and_shake
from envs.ring_bell_rhythm import ring_bell_rhythm
from envs.put_back_block import put_back_block
from envs.stamp_seal_cycled import stamp_seal_cycled
from envs.cycle_block import cycle_block
from envs.flip_cup_find_block import flip_cup_find_block
from envs.beat_block_hammer import beat_block_hammer

TASK_MAP = {
    "catch_bottle_and_shake": catch_bottle_and_shake,
    "ring_bell_rhythm": ring_bell_rhythm,
    "put_back_block": put_back_block,
    "stamp_seal_cycled": stamp_seal_cycled,
    "cycle_block": cycle_block,
    "flip_cup_find_block": flip_cup_find_block,
    "beat_block_hammer": beat_block_hammer
}

def run_task(task_name):
    print(f"\n[{task_name}] Initializing...")
    
    if task_name not in TASK_MAP:
        print(f"Unknown task: {task_name}")
        return

    robot_path = os.path.join(project_root, "assets/embodiments/aloha-agilex")
    config_path = os.path.join(robot_path, "config.yml")
    with open(config_path, "r") as f:
        robot_config = yaml.safe_load(f)
        
    debug_config = {
        "domain_randomization": {
            "random_background": False,
            "cluttered_table": False,
            "random_light": False,
            "random_table_height": 0,
            "random_head_camera_dis": 0
        },
        "task_name": task_name,
        "save_path": "debug_data",
        "save_data": False, # No need to save data
        "dual_arm": True,
        "left_robot_file": robot_path,
        "right_robot_file": robot_path,
        "left_embodiment_config": robot_config,
        "right_embodiment_config": robot_config,
        "dual_arm_embodied": True,
        "eval_mode": False,
        "camera": {
             "head_camera_type": "D435",
             "collect_head_camera": False, # Disable camera to speed up
             "wrist_camera_type": "D435",
             "collect_wrist_camera": False
        },
        "render_freq": 100 # Low render freq to speed up (or 0?) - Base_Task might expect it
    }

    env_class = TASK_MAP[task_name]
    env = env_class()
    
    # Initialize
    try:
        env.setup_demo(now_ep_num=0, seed=0, **debug_config)
    except Exception as e:
        print(f"Setup failed: {e}")
        return

    # Patch scene.step to count steps
    original_step = env.scene.step
    step_count = 0
    
    def counted_step():
        nonlocal step_count
        step_count += 1
        original_step()
        
    env.scene.step = counted_step
    
    print(f"[{task_name}] Running play_once...")
    try:
        env.play_once()
        print(f"[{task_name}] play_once finished.")
        print(f"[{task_name}] checking success...")
        is_success = env.check_success()
        print(f"[{task_name}] Success: {is_success}")
        print(f"[{task_name}] Total Steps: {step_count}")
    except Exception as e:
        print(f"[{task_name}] Error during execution: {e}")
    finally:
        env.close_env()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        tasks_to_run = sys.argv[1:]
    else:
        tasks_to_run = list(TASK_MAP.keys())

    for task in tasks_to_run:
        run_task(task)
