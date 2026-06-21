"""Franka Panda 演示视频录制脚本 - 使用imageio旧API"""

import sys
from pathlib import Path
import numpy as np
import mujoco

sys.path.insert(0, str(Path(__file__).parent))
from franka_controller import FrankaController


def record_demo(output_path="demo.mp4", fps=30, duration_sec=55):
    """录制演示视频"""
    import imageio
    
    ctrl = FrankaController()
    frames = []
    
    def capture(n=1):
        """捕获n帧"""
        for _ in range(n):
            mujoco.mj_step(ctrl.model, ctrl.data)
            frame = ctrl.render_frame()
            frames.append(frame.copy())
    
    print(f"开始录制... 目标{duration_sec}秒")
    
    # ===== 场景1: 初始展示 (0-3s) =====
    ctrl.reset()
    ctrl.gripper_control(0.04, steps=30)
    capture(fps * 3)
    
    # ===== 场景2: 工作空间扫描 (3-6s) =====
    scan_qpos = np.array([0.5, 0.3, 0.3, -1.57, 0.3, 1.57, -0.78])
    traj = ctrl.minimum_jerk_trajectory(ctrl.HOME_QPOS, scan_qpos, duration=2.0, num_points=fps*2)
    for pt in traj:
        ctrl.data.ctrl[:7] = pt.position
        capture(2)
    
    # ===== 场景3: Pick #1 (6-18s) =====
    obj1 = np.array([0.4, 0.1, 0.02])
    print("Pick #1...")
    
    # 移到上方
    pre = ctrl.pre_grasp_position(obj1)
    ctrl._move_to_cartesian_pos(pre, steps=60)
    capture(fps)
    
    # 下降
    plan = ctrl.compute_grasp_pose(obj1)
    ctrl._move_to_cartesian_pos(plan.grasp_pos, steps=60)
    capture(fps)
    
    # 抓取
    ctrl.gripper_control(0.015, steps=40)
    capture(fps)
    
    # 抬起
    ctrl._move_to_cartesian_pos(plan.lift_pos, steps=60)
    capture(fps * 2)
    
    # 移到放置位置
    place1 = np.array([0.25, 0.1, 0.05])
    above1 = place1.copy(); above1[2] += 0.2
    ctrl._move_to_cartesian_pos(above1, steps=60)
    ctrl._move_to_cartesian_pos(place1, steps=60)
    ctrl.gripper_control(0.04, steps=30)
    ctrl._move_to_cartesian_pos(above1, steps=40)
    capture(fps * 2)
    
    # ===== 场景4: Pick #2 (18-30s) =====
    obj2 = np.array([0.4, -0.1, 0.02])
    print("Pick #2...")
    
    pre = ctrl.pre_grasp_position(obj2)
    ctrl._move_to_cartesian_pos(pre, steps=60)
    capture(fps)
    
    plan2 = ctrl.compute_grasp_pose(obj2)
    ctrl._move_to_cartesian_pos(plan2.grasp_pos, steps=60)
    capture(fps)
    
    ctrl.gripper_control(0.015, steps=40)
    capture(fps)
    
    ctrl._move_to_cartesian_pos(plan2.lift_pos, steps=60)
    capture(fps * 2)
    
    place2 = np.array([0.1, 0.1, 0.05])
    above2 = place2.copy(); above2[2] += 0.2
    ctrl._move_to_cartesian_pos(above2, steps=60)
    ctrl._move_to_cartesian_pos(place2, steps=60)
    ctrl.gripper_control(0.04, steps=30)
    ctrl._move_to_cartesian_pos(above2, steps=40)
    capture(fps * 2)
    
    # ===== 场景5: Pick #3 + Stack (30-42s) =====
    obj3 = np.array([0.5, 0.0, 0.02])
    print("Pick #3 + Stack...")
    
    pre = ctrl.pre_grasp_position(obj3)
    ctrl._move_to_cartesian_pos(pre, steps=60)
    capture(fps)
    
    plan3 = ctrl.compute_grasp_pose(obj3)
    ctrl._move_to_cartesian_pos(plan3.grasp_pos, steps=60)
    ctrl.gripper_control(0.015, steps=40)
    ctrl._move_to_cartesian_pos(plan3.lift_pos, steps=60)
    capture(fps * 2)
    
    # 堆叠到obj1上面
    stack_pos = np.array([0.25, 0.1, 0.09])
    above_stack = stack_pos.copy(); above_stack[2] += 0.2
    ctrl._move_to_cartesian_pos(above_stack, steps=60)
    ctrl._move_to_cartesian_pos(stack_pos, steps=60)
    ctrl.gripper_control(0.04, steps=30)
    ctrl._move_to_cartesian_pos(above_stack, steps=40)
    capture(fps * 2)
    
    # ===== 场景6: 工作空间可视化 (42-48s) =====
    print("工作空间扫描...")
    for angle in np.linspace(0, 2*np.pi, 60):
        q = ctrl.HOME_QPOS.copy()
        q[0] = 0.3 * np.sin(angle)
        q[1] = 0.3 * np.cos(angle)
        ctrl.data.ctrl[:7] = q
        capture(1)
    
    # ===== 场景7: 回到首页 + 结束 (48-55s) =====
    home_traj = ctrl.minimum_jerk_trajectory(
        ctrl.data.qpos[:7], ctrl.HOME_QPOS, duration=2.0, num_points=fps*2
    )
    for pt in home_traj:
        ctrl.data.ctrl[:7] = pt.position
        capture(2)
    
    # 静止结束
    capture(fps * 3)
    
    # 写入视频
    print(f"写入视频... {len(frames)}帧")
    imageio.mimsave(output_path, frames, fps=fps, codec='libx264', quality=8)
    print(f"完成! {output_path} ({len(frames)/fps:.1f}秒)")


if __name__ == "__main__":
    import os
    output = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo.mp4")
    record_demo(output_path=output)
