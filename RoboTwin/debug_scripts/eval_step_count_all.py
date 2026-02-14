import os
import subprocess
import glob
import h5py
import numpy as np
import yaml

# List of tasks to evaluate
tasks = [
    "beat_block_hammer",
    "catch_bottle_and_shake",
    "ring_bell_rhythm",
    "stamp_seal_cycled",
    "flip_cup_find_block",
    "put_back_block",
    "cycle_block"
]

config_name = "demo_clean"
config_path = f"../task_config/{config_name}.yml"
output_file = "task_step_stats.txt"

# Read save_freq from config
save_freq = 15 # Default fallback
try:
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
        save_freq = config.get("save_freq", 15)
        print(f"已加载 save_freq: {save_freq} 来自 {config_path}")
except Exception as e:
    print(f"读取配置文件错误 {config_path}: {e}, 使用默认 save_freq={save_freq}")

print(f"开始评估 {len(tasks)} 个任务...")

with open(output_file, "w") as f_out:
    # Adjusted headers
    # "HDF5 Frames": Raw frames in .hdf5 file (sampled at save_freq)
    # "250Hz Steps": Estimated total simulation steps (HDF5 Frames * save_freq)
    # "30Hz Frames": Estimated frames if sampling at 30Hz (250Hz Steps * 0.12)
    header = f"{'任务名称':<25} | {'HDF5帧数(平均)':<20} | {'预估250Hz步数':<20} | {'预估30Hz帧数':<20}\n"
    print(header)
    f_out.write(header)
    f_out.write("-" * 90 + "\n")

    for task in tasks:
        # Run data collection (assuming 5 episodes as per current config)
        print(f"正在处理任务: {task}...", flush=True)
        cmd = ["bash", "collect_data.sh", task, config_name, "0"]
        try:
             # Run quietly
            subprocess.run(cmd, cwd="..", check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # subprocess.run(cmd, check=True)
        except subprocess.CalledProcessError as e:
            print(f"任务 {task} 数据收集出错: {e}")
            continue

        # Find HDF5 files
        data_dir = os.path.join("../data", task, config_name, "data")
        hdf5_files = glob.glob(os.path.join(data_dir, "*.hdf5"))
        
        if not hdf5_files:
            print(f"未找到任务 {task} 的数据")
            continue

        raw_frames = []
        for file_path in hdf5_files:
            try:
                with h5py.File(file_path, 'r') as f:
                    if 'joint_action/vector' in f:
                        raw_frames.append(f['joint_action/vector'].shape[0])
                    elif 'qpos' in f:
                        raw_frames.append(f['qpos'].shape[0]) # Some older formats
                    elif 'action' in f: 
                         raw_frames.append(f['action'].shape[0])
            except Exception as e:
                print(f"读取文件 {file_path} 出错: {e}")

        if raw_frames:
            avg_raw_frames = np.mean(raw_frames)
            
            # Calculations
            est_total_steps_250hz = avg_raw_frames * save_freq
            est_frames_30hz = est_total_steps_250hz * 0.12 # 30 / 250 = 0.12

            line = f"{task:<25} | {avg_raw_frames:<20.1f} | {int(est_total_steps_250hz):<20} | {int(est_frames_30hz):<20}\n"
            print(line.strip())
            f_out.write(line)
        else:
            print(f"未找到 {task} 的有效帧数据")

print(f"\n评估完成。结果已保存至 {output_file}")
