"""在Windows上用MuJoCo GUI录制视频"""
import sys, os, time, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 添加vendor路径
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)

import mujoco
from mujoco import viewer

def main():
    model_path = os.path.join(project_root, "vendor", "mujoco_menagerie", "franka_emika_panda", "scene.xml")
    print(f"模型路径: {model_path}")
    
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    print("启动MuJoCo GUI窗口...")
    print("窗口会在你的桌面上弹出！")
    print("按Ctrl+C关闭")
    
    # 启动被动viewer（可以自己控制）
    v = viewer.launch_passive(model, data)
    
    # 运行仿真
    for i in range(10000):
        mujoco.mj_step(model, data)
        v.sync()
        time.sleep(0.02)  # 50fps
        
        if i % 100 == 0:
            print(f"  帧 {i}/10000")
    
    v.close()
    print("完成!")

if __name__ == "__main__":
    main()
