import sys
import os
import yaml
import numpy as np
import argparse

# Add project root to path and change working directory
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.append(project_root)

# Change to project root directory before importing
# This is necessary because the envs module expects to be run from the project root
os.chdir(project_root)

from envs.flip_cup_find_block import flip_cup_find_block

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Debug render flip_cup_find_block environment')
    parser.add_argument('--episode', type=int, default=0, help='Episode number (default: 0)')
    parser.add_argument('--random', action='store_true', help='Use random block generation mode')
    args = parser.parse_args()
    
    print(f"=== 测试配置 ===")
    print(f"Episode 编号: {args.episode}")
    print(f"生成模式: {'随机' if args.random else '顺序'}")
    print(f"================\n")
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
        "task_name": "flip_cup_find_block",
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
        "render_freq": 1,
        "random_block_order": args.random  # 使用命令行参数
    }

    print("Initializing environment...")
    env = flip_cup_find_block()
    # setup_demo will initialize the viewer if render_freq > 0
    # 随机模式使用随机整数作为seed，顺序模式使用0(可复现)
    seed_value = np.random.randint(0, 100000) if args.random else 0
    env.setup_demo(now_ep_num=args.episode, seed=seed_value, **debug_config)
    
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
    
    # 保持窗口打开,直到用户按下回车键
    print("\n" + "="*50)
    print("场景已加载完成!")
    print("按 Enter 键关闭窗口...")
    print("="*50)
    input()  # 等待用户按下回车键
    
    env.close_env()
    print("Done.")

if __name__ == "__main__":
    main()
