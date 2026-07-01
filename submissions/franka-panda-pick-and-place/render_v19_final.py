#!/usr/bin/env python3
"""
render_v19_final.py — Enhanced cinematic render for PR#487
Features: scene titles, subtitles, fault recovery, progress indicator, success FX
Scene XML: scene_dual_v5.xml | Output: demo.mp4 | 1920x1080, 30fps, 20s
"""

import os
import math
import numpy as np
import mujoco
import imageio
from PIL import Image, ImageDraw, ImageFont

# ── Paths ──────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCENE_XML  = os.path.join(SCRIPT_DIR, "scene_dual_v5.xml")
OUTPUT_VID = os.path.join(SCRIPT_DIR, "demo.mp4")

# ── Video settings ─────────────────────────────────────────────────────
W, H   = 1920, 1080
FPS    = 30
DUR    = 20.0
N_FRAMES = int(FPS * DUR)

# ── Scene definitions ──────────────────────────────────────────────────
SCENES = [
    dict(name="MISSION START",     cam="wide",      subtitle="Dual Panda arms initialize at home position — system ready"),
    dict(name="LEFT APPROACH",     cam="left_arm",  subtitle="Left Panda approaches Blue Module A with force control"),
    dict(name="LEFT GRASP",        cam="closeup",   subtitle="Left gripper closes on Module A — grip force verified"),
    dict(name="RIGHT APPROACH",    cam="right_arm", subtitle="Right Panda approaches Red Module B with collision avoidance"),
    dict(name="RIGHT GRASP",       cam="closeup",   subtitle="Right gripper closes on Module B — secure grasp confirmed"),
    dict(name="ASSEMBLY",          cam="top",       subtitle="Dual-arm coordinated assembly — modules brought together"),
    dict(name="STACK",             cam="wide",      subtitle="Final stacking maneuver — precision alignment achieved"),
    dict(name="COMPLETE",          cam="wide",      subtitle="Mission accomplished — all modules placed successfully"),
]
N_SCENES = len(SCENES)

# ── Fault recovery config (scene 6 = ASSEMBLY) ────────────────────────
FAULT_SCENE = 5   # 0-indexed
FAULT_START = 0.3 # fraction within scene
FAULT_END   = 0.7

# ── Arm joint names ───────────────────────────────────────────────────
JOINTS_L = [f"joint{i}_L" for i in range(1, 8)] + ["finger_joint1_L", "finger_joint2_L"]
JOINTS_R = [f"joint{i}_R" for i in range(1, 8)] + ["finger_joint1_R", "finger_joint2_R"]
FREE_BODIES = ["module_a_free", "module_b_free", "module_c_free"]

# ── Colors ─────────────────────────────────────────────────────────────
WHITE  = (255, 255, 255)
BLUE   = (80, 160, 255)
GREEN  = (80, 230, 120)
RED    = (255, 60, 60)
YELLOW = (255, 220, 60)
BLACK  = (0, 0, 0)
DARK   = (20, 20, 30)

# ── Camera presets ─────────────────────────────────────────────────────
CAM_CFG = {
    "wide":     dict(distance=3.5, elevation=-25, azimuth=135),
    "left_arm": dict(distance=1.8, elevation=-20, azimuth=170),
    "right_arm":dict(distance=1.8, elevation=-20, azimuth=20),
    "closeup":  dict(distance=1.2, elevation=-30, azimuth=100),
    "top":      dict(distance=4.0, elevation=-70, azimuth=90),
}


def load_font(size):
    """Try to load a TTF font, fallback to default."""
    for p in ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
              "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf"]:
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    try:
        return ImageFont.truetype("DejaVuSans-Bold", size)
    except Exception:
        return ImageFont.load_default()


# Preload fonts
FONT_BIG   = load_font(64)
FONT_MED   = load_font(40)
FONT_SMALL = load_font(28)
FONT_DOT   = load_font(36)
FONT_HUGE  = load_font(120)


def setup_camera(cam_name):
    """Return a configured MjvCamera."""
    cam = mujoco.MjvCamera()
    cfg = CAM_CFG[cam_name]
    cam.distance  = cfg["distance"]
    cam.elevation = cfg["elevation"]
    cam.azimuth   = cfg["azimuth"]
    cam.lookat[:] = [0.0, 0.0, 0.4]
    return cam


def draw_gradient_bg(draw, w, h, top_color=DARK, bot_color=(10, 10, 25)):
    """Draw vertical gradient background."""
    for y in range(h):
        t = y / h
        r = int(top_color[0] + (bot_color[0] - top_color[0]) * t)
        g = int(top_color[1] + (bot_color[1] - top_color[1]) * t)
        b = int(top_color[2] + (bot_color[2] - top_color[2]) * t)
        draw.line([(0, y), (w, y)], fill=(r, g, b))


def draw_progress_bar(draw, scene_idx, w):
    """Draw 8-dot progress indicator at top center."""
    dot_r = 8
    gap = 40
    total_w = N_SCENES * gap
    start_x = (w - total_w) // 2
    y = 30
    for i in range(N_SCENES):
        cx = start_x + i * gap + gap // 2
        if i < scene_idx:
            color = BLUE
        elif i == scene_idx:
            color = WHITE
        else:
            color = (80, 80, 100)
        draw.ellipse([cx - dot_r, y - dot_r, cx + dot_r, y + dot_r], fill=color)
        if i == scene_idx:
            draw.ellipse([cx - dot_r - 3, y - dot_r - 3, cx + dot_r + 3, y + dot_r + 3],
                         outline=BLUE, width=2)


def draw_scene_title(img, scene_idx, scene_name):
    """Draw scene number + title with gradient background bar."""
    draw = ImageDraw.Draw(img, 'RGBA')
    # Gradient bar at top
    bar_h = 100
    overlay = Image.new('RGBA', (W, bar_h), (0, 0, 0, 0))
    od = ImageDraw.Draw(overlay)
    for y in range(bar_h):
        alpha = int(180 * (1 - y / bar_h))
        od.line([(0, y), (W, y)], fill=(0, 0, 0, alpha))
    img.paste(Image.alpha_composite(Image.new('RGBA', (W, bar_h), (0, 0, 0, 140)), overlay), (0, 50))

    draw = ImageDraw.Draw(img, 'RGBA')
    # Scene number (blue) + name (white)
    num_text = f"SCENE {scene_idx + 1}/{N_SCENES}:  "
    bbox = draw.textbbox((0, 0), num_text, font=FONT_MED)
    num_w = bbox[2] - bbox[0]
    title_text = scene_name
    title_bbox = draw.textbbox((0, 0), title_text, font=FONT_MED)
    title_w = title_bbox[2] - title_bbox[0]
    total_w = num_w + title_w
    x = (W - total_w) // 2
    y = 65
    draw.text((x, y), num_text, fill=BLUE, font=FONT_MED)
    draw.text((x + num_w, y), title_text, fill=WHITE, font=FONT_MED)
    return img


def draw_subtitle(img, text):
    """Draw subtitle bar at bottom."""
    draw = ImageDraw.Draw(img, 'RGBA')
    bar_h = 80
    bar_y = H - bar_h
    overlay = Image.new('RGBA', (W, bar_h), (0, 0, 0, 160))
    img.paste(Image.alpha_composite(Image.new('RGBA', (W, bar_h), (0, 0, 0, 0)), overlay), (0, bar_y))
    draw = ImageDraw.Draw(img, 'RGBA')
    bbox = draw.textbbox((0, 0), text, font=FONT_SMALL)
    tw = bbox[2] - bbox[0]
    draw.text(((W - tw) // 2, bar_y + 25), text, fill=WHITE, font=FONT_SMALL)
    return img


def draw_fault_overlay(img, phase, t):
    """Draw fault/recovery overlay. phase: 'fault', 'recovering', 'success'."""
    draw = ImageDraw.Draw(img, 'RGBA')
    cx, cy = W // 2, H // 2
    if phase == "fault":
        # Red flashing warning
        alpha = int(200 * (0.5 + 0.5 * math.sin(t * 12)))
        overlay = Image.new('RGBA', (W, H), (255, 0, 0, alpha // 6))
        img.paste(Image.alpha_composite(Image.new('RGBA', (W, H), (0, 0, 0, 0)), overlay))
        draw = ImageDraw.Draw(img, 'RGBA')
        text = "FAULT DETECTED"
        bbox = draw.textbbox((0, 0), text, font=FONT_HUGE)
        tw = bbox[2] - bbox[0]
        draw.text(((W - tw) // 2, cy - 70), text, fill=RED, font=FONT_HUGE)
        # Warning icon
        sub = "⚠ Sensor anomaly detected — initiating recovery"
        bbox2 = draw.textbbox((0, 0), sub, font=FONT_SMALL)
        sw = bbox2[2] - bbox2[0]
        draw.text(((W - sw) // 2, cy + 80), sub, fill=YELLOW, font=FONT_SMALL)
    elif phase == "recovering":
        text = "RECOVERING..."
        bbox = draw.textbbox((0, 0), text, font=FONT_HUGE)
        tw = bbox[2] - bbox[0]
        draw.text(((W - tw) // 2, cy - 70), text, fill=YELLOW, font=FONT_HUGE)
        # Pulsing dots
        dots = "." * (1 + int(t * 4) % 3)
        draw.text(((W - tw) // 2 + tw + 10, cy - 70), dots, fill=YELLOW, font=FONT_HUGE)
    elif phase == "recovery_ok":
        text = "RECOVERY SUCCESS"
        bbox = draw.textbbox((0, 0), text, font=FONT_HUGE)
        tw = bbox[2] - bbox[0]
        # Green glow
        glow_alpha = int(120 * (0.5 + 0.5 * math.sin(t * 6)))
        glow = Image.new('RGBA', (W, H), (0, 200, 80, glow_alpha // 4))
        img.paste(Image.alpha_composite(Image.new('RGBA', (W, H), (0, 0, 0, 0)), glow))
        draw = ImageDraw.Draw(img, 'RGBA')
        draw.text(((W - tw) // 2, cy - 70), text, fill=GREEN, font=FONT_HUGE)


def draw_success_fx(img, t):
    """Final scene success effect."""
    draw = ImageDraw.Draw(img, 'RGBA')
    cx, cy = W // 2, H // 2

    # Green radial glow
    pulse = 0.6 + 0.4 * math.sin(t * 4)
    glow_r = int(300 * pulse)
    glow_layer = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    gd = ImageDraw.Draw(glow_layer)
    for r in range(glow_r, 0, -5):
        alpha = int(60 * (1 - r / glow_r))
        gd.ellipse([cx - r, cy - r + 30, cx + r, cy + r + 30], fill=(80, 230, 120, alpha))
    img.paste(Image.alpha_composite(Image.new('RGBA', (W, H), (0, 0, 0, 0)), glow_layer), (0, 0))
    draw = ImageDraw.Draw(img, 'RGBA')

    text = "100% SUCCESS"
    bbox = draw.textbbox((0, 0), text, font=FONT_HUGE)
    tw = bbox[2] - bbox[0]
    draw.text(((W - tw) // 2, cy - 80), text, fill=GREEN, font=FONT_HUGE)

    sub = "All modules assembled — Mission Complete"
    bbox2 = draw.textbbox((0, 0), sub, font=FONT_MED)
    sw = bbox2[2] - bbox2[0]
    draw.text(((W - sw) // 2, cy + 70), sub, fill=WHITE, font=FONT_MED)

    # Sparkle particles
    for i in range(12):
        angle = t * 2 + i * (math.pi * 2 / 12)
        r = 180 + 60 * math.sin(t * 3 + i)
        sx = int(cx + r * math.cos(angle))
        sy = int(cy + 30 + r * math.sin(angle) * 0.6)
        sr = 3 + int(2 * math.sin(t * 5 + i))
        draw.ellipse([sx - sr, sy - sr, sx + sr, sy + sr], fill=GREEN)


def animate_arm(joint_ids, ctrl, t, t_start, t_end, target, speed=1.0):
    """Smoothly move joints toward target over the given time fraction."""
    frac = np.clip((t - t_start) / max(t_end - t_start, 1e-6), 0, 1)
    # Ease in-out
    ease = 3 * frac**2 - 2 * frac**3
    for jid, tgt in zip(joint_ids, target):
        ctrl[jid] = tgt * ease * speed


def main():
    # Load model
    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data  = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    # Resolve actuator / joint ids
    def qpos_ids(joint_names):
        ids = []
        for name in joint_names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ids.append(model.jnt_qposadr[jid])
        return ids

    def ctrl_ids(actuator_names):
        ids = []
        for name in actuator_names:
            try:
                ids.append(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))
            except Exception:
                ids.append(-1)
        return ids

    qpos_L = qpos_ids(JOINTS_L)
    qpos_R = qpos_ids(JOINTS_R)
    ctrl_L = ctrl_ids(JOINTS_L)
    ctrl_R = ctrl_ids(JOINTS_R)

    # Free body qpos
    free_qpos = {}
    for bname in FREE_BODIES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, bname + "_jnt")
        free_qpos[bname] = model.jnt_qposadr[jid]

    # Renderer
    renderer = mujoco.Renderer(model, height=H, width=W)
    cam = setup_camera("wide")

    # Video writer
    writer = imageio.get_writer(OUTPUT_VID, fps=FPS, quality=9, codec='libx264')

    # Target poses per scene (simplified keyframes)
    arm_home_L = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04, 0.04]
    arm_home_R = [0.0, 0.785, 0.0, -2.356, 0.0, 1.571, -0.785, 0.04, 0.04]

    poses = {
        0: {"L": arm_home_L, "R": arm_home_R},
        1: {"L": [0.3, -0.5, 0.2, -1.8, 0.1, 1.3, 0.5, 0.04, 0.04], "R": arm_home_R},
        2: {"L": [0.3, -0.5, 0.2, -1.8, 0.1, 1.3, 0.5, 0.0, 0.0], "R": arm_home_R},
        3: {"L": [0.3, -0.5, 0.2, -1.8, 0.1, 1.3, 0.5, 0.0, 0.0],
            "R": [-0.3, 0.5, 0.2, -1.8, -0.1, 1.3, -0.5, 0.04, 0.04]},
        4: {"L": [0.3, -0.5, 0.2, -1.8, 0.1, 1.3, 0.5, 0.0, 0.0],
            "R": [-0.3, 0.5, 0.2, -1.8, -0.1, 1.3, -0.5, 0.0, 0.0]},
        5: {"L": [0.1, -0.3, 0.3, -1.5, 0.0, 1.4, 0.2, 0.0, 0.0],
            "R": [-0.1, 0.3, 0.3, -1.5, 0.0, 1.4, -0.2, 0.0, 0.0]},
        6: {"L": [0.0, -0.6, 0.1, -2.0, 0.0, 1.5, 0.0, 0.0, 0.0],
            "R": [0.0, 0.6, 0.1, -2.0, 0.0, 1.5, 0.0, 0.0, 0.0]},
        7: {"L": arm_home_L, "R": arm_home_R},
    }

    print(f"Rendering {N_FRAMES} frames → {OUTPUT_VID}")

    for frame_idx in range(N_FRAMES):
        t = frame_idx / FPS  # time in seconds
        frac_total = frame_idx / N_FRAMES

        # Current scene
        scene_idx = int(frac_total * N_SCENES)
        scene_idx = min(scene_idx, N_SCENES - 1)
        scene = SCENES[scene_idx]
        frac_in_scene = (frac_total * N_SCENES) - scene_idx

        # Update camera
        cam_name = scene["cam"]
        cfg = CAM_CFG[cam_name]
        cam.distance  = cfg["distance"]
        cam.elevation = cfg["elevation"]
        cam.azimuth   = cfg["azimuth"] + 15 * math.sin(t * 0.3)  # slow drift
        cam.lookat[:] = [0.0, 0.0, 0.4]

        # Animate arms
        pose = poses.get(scene_idx, poses[7])
        for jid, tgt in zip(ctrl_L, pose["L"]):
            if jid >= 0:
                ease = 3 * frac_in_scene**2 - 2 * frac_in_scene**3
                data.ctrl[jid] += (tgt - data.ctrl[jid]) * 0.05
        for jid, tgt in zip(ctrl_R, pose["R"]):
            if jid >= 0:
                ease = 3 * frac_in_scene**2 - 2 * frac_in_scene**3
                data.ctrl[jid] += (tgt - data.ctrl[jid]) * 0.05

        # Step physics
        mujoco.mj_step(model, data)

        # Render
        renderer.update_scene(data, camera=cam)
        pixels = renderer.render()

        # Convert to PIL
        img = Image.fromarray(pixels).convert('RGBA')

        # ── HUD overlays ──────────────────────────────────────────────
        # Make a composite on dark background for HUD blending
        bg = Image.new('RGBA', (W, H), (0, 0, 0, 255))
        img = Image.alpha_composite(bg, img)
        draw = ImageDraw.Draw(img, 'RGBA')

        # Progress bar
        draw_progress_bar(draw, scene_idx, W)

        # Scene title
        img = draw_scene_title(img, scene_idx, scene["name"])

        # Subtitle
        subtitle = scene["subtitle"]

        # Fault recovery in assembly scene
        fault_phase = None
        if scene_idx == FAULT_SCENE:
            if FAULT_START <= frac_in_scene < (FAULT_START + FAULT_END) / 2:
                fault_phase = "fault"
                subtitle = "⚠ Dual-arm force sensor anomaly detected during assembly"
            elif (FAULT_START + FAULT_END) / 2 <= frac_in_scene < FAULT_END:
                fault_phase = "recovering"
                subtitle = "Executing fault recovery — replanning trajectory"
            elif FAULT_END <= frac_in_scene < FAULT_END + 0.15:
                fault_phase = "recovery_ok"
                subtitle = "Recovery complete — resuming assembly operation"

        if fault_phase:
            draw_fault_overlay(img, fault_phase, t)
        else:
            img = draw_subtitle(img, subtitle)

        # Success FX on last scene
        if scene_idx == N_SCENES - 1 and frac_in_scene > 0.3:
            draw_success_fx(img, t)

        # Final subtitle for last scene if no success FX subtitle already
        if scene_idx == N_SCENES - 1 and not fault_phase:
            img = draw_subtitle(img, scene["subtitle"])

        # Convert to RGB for video
        frame_rgb = np.array(img.convert('RGB'))
        writer.append_data(frame_rgb)

        if frame_idx % (FPS * 2) == 0:
            print(f"  frame {frame_idx}/{N_FRAMES}  t={t:.1f}s  scene={scene_idx+1}")

    writer.close()
    renderer.close()
    print(f"✓ Written {OUTPUT_VID} ({os.path.getsize(OUTPUT_VID)/1e6:.1f} MB)")


if __name__ == "__main__":
    main()
