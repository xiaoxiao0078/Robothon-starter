"""录制MuJoCo真实渲染的demo视频"""
import sys, os, time, numpy as np

# 添加路径
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mujoco
from franka_controller import FrankaController

def record_real_demo(output_path="demo_real.mp4", fps=30):
    """用MuJoCo Renderer录制真实渲染视频"""
    import imageio
    
    ctrl = FrankaController()
    frames = []
    
    def capture(n=1):
        for _ in range(n):
            mujoco.mj_step(ctrl.model, ctrl.data)
            frame = ctrl.renderer.render()
            frames.append(frame.copy())
    
    print("开始录制真实渲染视频...")
    
    # 1. 初始展示 (3秒)
    ctrl.reset()
    ctrl.gripper_control(0.04, steps=30)
    capture(fps * 3)
    print(f"  初始展示: {fps*3}帧")
    
    # 2. 移动到观察位 (2秒)
    scan = np.array([0.3, 0.3, 0.3, -1.57, 0.3, 1.57, -0.78])
    traj = ctrl.minimum_jerk_trajectory(ctrl.HOME_QPOS, scan, duration=2.0, num_points=fps*2)
    for pt in traj:
        ctrl.data.ctrl[:7] = pt.position
        capture(2)
    print("  观察位移动完成")
    
    # 3. Pick #1 (6秒)
    obj1 = np.array([0.4, 0.1, 0.02])
    pre = ctrl.pre_grasp_position(obj1)
    ctrl._move_to_cartesian_pos(pre, steps=60)
    capture(fps)
    
    plan = ctrl.compute_grasp_pose(obj1)
    ctrl._move_to_cartesian_pos(plan.grasp_pos, steps=60)
    ctrl.gripper_control(0.015, steps=40)
    capture(fps)
    
    ctrl._move_to_cartesian_pos(plan.lift_pos, steps=60)
    capture(fps)
    
    place1 = np.array([0.25, 0.1, 0.05])
    above1 = place1.copy(); above1[2] += 0.2
    ctrl._move_to_cartesian_pos(above1, steps=60)
    ctrl._move_to_cartesian_pos(place1, steps=60)
    ctrl.gripper_control(0.04, steps=30)
    ctrl._move_to_cartesian_pos(above1, steps=40)
    capture(fps)
    print(f"  Pick #1完成: 总{len(frames)}帧")
    
    # 4. Pick #2 (6秒)
    obj2 = np.array([0.4, -0.1, 0.02])
    pre = ctrl.pre_grasp_position(obj2)
    ctrl._move_to_cartesian_pos(pre, steps=60)
    capture(fps)
    
    plan2 = ctrl.compute_grasp_pose(obj2)
    ctrl._move_to_cartesian_pos(plan2.grasp_pos, steps=60)
    ctrl.gripper_control(0.015, steps=40)
    ctrl._move_to_cartesian_pos(plan2.lift_pos, steps=60)
    capture(fps)
    
    place2 = np.array([0.1, 0.1, 0.05])
    above2 = place2.copy(); above2[2] += 0.2
    ctrl._move_to_cartesian_pos(above2, steps=60)
    ctrl._move_to_cartesian_pos(place2, steps=60)
    ctrl.gripper_control(0.04, steps=30)
    ctrl._move_to_cartesian_pos(above2, steps=40)
    capture(fps)
    print(f"  Pick #2完成: 总{len(frames)}帧")
    
    # 5. Pick #3 + Stack (6秒)
    obj3 = np.array([0.5, 0.0, 0.02])
    pre = ctrl.pre_grasp_position(obj3)
    ctrl._move_to_cartesian_pos(pre, steps=60)
    capture(fps)
    
    plan3 = ctrl.compute_grasp_pose(obj3)
    ctrl._move_to_cartesian_pos(plan3.grasp_pos, steps=60)
    ctrl.gripper_control(0.015, steps=40)
    ctrl._move_to_cartesian_pos(plan3.lift_pos, steps=60)
    capture(fps)
    
    stack_pos = np.array([0.25, 0.1, 0.09])
    above_s = stack_pos.copy(); above_s[2] += 0.2
    ctrl._move_to_cartesian_pos(above_s, steps=60)
    ctrl._move_to_cartesian_pos(stack_pos, steps=60)
    ctrl.gripper_control(0.04, steps=30)
    ctrl._move_to_cartesian_pos(above_s, steps=40)
    capture(fps)
    print(f"  Pick #3+Stack完成: 总{len(frames)}帧")
    
    # 6. 工作空间扫描 (3秒)
    for angle in np.linspace(0, 2*np.pi, fps*3):
        q = ctrl.HOME_QPOS.copy()
        q[0] = 0.3 * np.sin(angle)
        q[1] = 0.3 * np.cos(angle)
        ctrl.data.ctrl[:7] = q
        capture(1)
    print(f"  扫描完成: 总{len(frames)}帧")
    
    # 7. 回首页+结束 (4秒)
    home_traj = ctrl.minimum_jerk_trajectory(ctrl.data.qpos[:7], ctrl.HOME_QPOS, duration=2.0, num_points=fps*2)
    for pt in home_traj:
        ctrl.data.ctrl[:7] = pt.position
        capture(2)
    capture(fps * 2)
    
    # 写入视频
    print(f"\n写入视频... {len(frames)}帧")
    imageio.mimsave(output_path, frames, fps=fps, codec='libx264')
    print(f"完成! {output_path} ({len(frames)/fps:.1f}秒, {os.path.getsize(output_path)/1024/1024:.1f}MB)")

if __name__ == "__main__":
    output = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_real.mp4")
    record_real_demo(output)
