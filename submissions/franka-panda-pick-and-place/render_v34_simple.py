#!/usr/bin/env python3
"""
v34 简化版视频渲染 — 快速生成demo
"""

import mujoco
import numpy as np
import os
from PIL import Image, ImageDraw, ImageFont

# 配置
XML = "scene_dual_v5.xml"
OUT = "dual_arm_v34_demo.mp4"
FPS = 30
W, H = 1920, 1080

# 加载模型
model = mujoco.MjModel.from_xml_path(XML)
data = mujoco.MjData(model)
rend = mujoco.Renderer(model, height=H, width=W)

# 工具函数
def qa(name):
    return model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]

def da(name):
    return model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]

def body_id(name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)

# 关节配置
L_Q = [qa(f"joint{i}_L") for i in range(1, 8)]
L_D = [da(f"joint{i}_L") for i in range(1, 8)]
L_CI = list(range(0, 7))

R_Q = [qa(f"joint{i}_R") for i in range(1, 8)]
R_D = [da(f"joint{i}_R") for i in range(1, 8)]
R_CI = list(range(8, 15))

L_FQ = [qa("finger_joint1_L"), qa("finger_joint2_L")]
R_FQ = [qa("finger_joint1_R"), qa("finger_joint2_R")]
L_FI = [7]
R_FI = [15]

hL = body_id("hand_L")
hR = body_id("hand_R")

# Home位置
HOME_L = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
HOME_R = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]

# IK求解
def solve_ik(target, qi, di, bid, iters=200):
    for _ in range(iters):
        mujoco.mj_forward(model, data)
        err = target - data.xpos[bid]
        if np.linalg.norm(err) < 0.005:
            return True
        J = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, J, None, data.xpos[bid].copy(), bid)
        Ja = np.zeros((3, 7))
        for i in range(7):
            Ja[:, i] = J[:, di[i]]
        dq = np.linalg.solve(Ja.T @ Ja + 0.02 * np.eye(7), Ja.T @ err)
        for i in range(7):
            data.qpos[qi[i]] += dq[i] * 0.3
    return False

def get_ik(target):
    mujoco.mj_resetData(model, data)
    for _ in range(50):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    solve_ik(target, L_Q, L_D, hL)
    return [data.qpos[L_Q[i]] for i in range(7)]

# 移动关节
def move_joints(qi, ci, target, steps=30):
    start = np.array([data.qpos[qi[i]] for i in range(7)])
    for step in range(steps):
        t = step / steps
        t_smooth = t * t * (3 - 2 * t)
        for i in range(7):
            data.qpos[qi[i]] = start[i] + (target[i] - start[i]) * t_smooth
            data.ctrl[ci[i]] = data.qpos[qi[i]]
        mujoco.mj_step(model, data)

# 控制夹爪
def control_gripper(fi, width, steps=20):
    for _ in range(steps):
        data.ctrl[fi[0]] = width
        mujoco.mj_step(model, data)

# 渲染帧
def render_frame():
    rend.update_scene(data)
    return rend.render()

# 添加HUD
def add_hud(frame, text, progress):
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    
    # 进度条
    bar_y = H - 40
    draw.rectangle([50, bar_y, W-50, bar_y+15], fill=(50, 50, 50))
    draw.rectangle([50, bar_y, 50 + int((W-100) * progress), bar_y+15], fill=(0, 200, 100))
    
    # 文字
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
    except:
        font = ImageFont.load_default()
    
    draw.text((50, 20), text, fill=(255, 255, 255), font=font)
    
    return np.array(img)

# 主函数
def main():
    import imageio
    
    print("Creating demo video...")
    
    # 重置
    mujoco.mj_resetData(model, data)
    for _ in range(50):
        mujoco.mj_step(model, data)
    
    frames = []
    
    # 模块位置
    module_positions = [
        np.array([0.15, 0.0, 0.44]),
        np.array([0.0, -0.1, 0.44]),
        np.array([-0.15, 0.1, 0.44])
    ]
    
    # 移动到home
    move_joints(L_Q, L_CI, HOME_L, steps=30)
    move_joints(R_Q, R_CI, HOME_R, steps=30)
    for _ in range(20):
        frames.append(render_frame())
    
    # 装配3个模块
    for i, module_pos in enumerate(module_positions):
        # 计算IK
        pre_grasp = module_pos.copy()
        pre_grasp[2] += 0.15
        grasp_pos = module_pos.copy()
        grasp_pos[2] += 0.02
        
        qL_pre = get_ik(pre_grasp)
        qL_grasp = get_ik(grasp_pos)
        
        # 移动到预抓取
        move_joints(L_Q, L_CI, qL_pre, steps=30)
        for _ in range(10):
            frames.append(render_frame())
        
        # 下降
        move_joints(L_Q, L_CI, qL_grasp, steps=20)
        for _ in range(10):
            frames.append(render_frame())
        
        # 抓取
        control_gripper(L_FI, 0.0, steps=15)
        for _ in range(10):
            frames.append(render_frame())
        
        # 抬起
        lift_pos = module_pos.copy()
        lift_pos[2] += 0.2
        qL_lift = get_ik(lift_pos)
        move_joints(L_Q, L_CI, qL_lift, steps=30)
        for _ in range(10):
            frames.append(render_frame())
        
        # 移动到装配区
        assembly_pos = np.array([0.0, 0.0, 0.5 + i * 0.05])
        qL_assembly = get_ik(assembly_pos)
        move_joints(L_Q, L_CI, qL_assembly, steps=40)
        for _ in range(10):
            frames.append(render_frame())
        
        # 释放
        control_gripper(L_FI, 0.04, steps=10)
        for _ in range(10):
            frames.append(render_frame())
    
    # 最终展示
    for _ in range(30):
        frames.append(render_frame())
    
    # 添加HUD
    total_frames = len(frames)
    for i in range(total_frames):
        progress = i / total_frames
        text = f"Module Assembly: {min(3, int(i / (total_frames/3)))}/3 | UAHP Closed-Loop Control"
        frames[i] = add_hud(frames[i], text, progress)
    
    # 保存视频
    imageio.mimsave(OUT, frames, fps=FPS)
    print(f"Video saved: {OUT} ({len(frames)} frames)")

if __name__ == "__main__":
    main()
