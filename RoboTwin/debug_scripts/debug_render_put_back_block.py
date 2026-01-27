import sys
import os
import yaml
import numpy as np

# Add project root to path
# Assuming this script is in RoboTwin/debug_scripts/
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.append(project_root)

from envs.put_back_block import put_back_block

def main():
    # 1. Load config
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
        "task_name": "put_back_block",
        "save_path": "debug_data",
        "save_data": True,
        "dual_arm": True,
        "left_robot_file": robot_path,
        "right_robot_file": robot_path,
        "left_embodiment_config": robot_config,
        "right_embodiment_config": robot_config,
        "dual_arm_embodied": True,
        "eval_mode": False,
        "camera": {
             "head_camera_type": "D435",
             "collect_head_camera": True,
             "wrist_camera_type": "D435",
             "collect_wrist_camera": True
        },
        "render_freq": 1 
    }

    print("Initializing environment...")
    env = put_back_block()
    # setup_demo will initialize the viewer if render_freq > 0
    env.setup_demo(now_ep_num=0, seed=0, **debug_config)
    
    print("Running task sequence...")
    try:
        env.play_once()
    except Exception as e:
        print(f"Error during play_once: {e}")
    
    # Check if task was successful
    print("\nChecking success...")
    success = env.check_success()
    if success:
        print("✓ Task completed successfully!")
    else:
        print("✗ Task failed.")
    
    env.close_env()
    print("Done.")

if __name__ == "__main__":
    main()
