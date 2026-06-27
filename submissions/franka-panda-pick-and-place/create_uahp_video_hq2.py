"""
UAHP视频 - 高质量版 30秒 30fps 150dpi
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import json
import os
import subprocess
from dual_arm.uahp import UAHPController, HandoffStrategy


def generate_frames():
    controller = UAHPController()
    duration = 30.0
    fps = 15  # 15fps
    dt = 1.0 / fps
    total_frames = int(duration / dt)  # 450帧
    
    os.makedirs('uahp_frames_hq', exist_ok=True)
    
    timestamps = []
    hcs_values = []
    strategies = []
    components = {'grasp': [], 'velocity': [], 'alignment': [], 'b_readiness': []}
    
    strategy_colors = {
        'fast_transfer': '#2ecc71',
        'slow_align': '#3498db',
        'pause_replan': '#f39c12',
        'emergency_stop': '#e74c3c'
    }
    
    print(f"生成 {total_frames} 帧...")
    
    for frame in range(total_frames):
        t = frame * dt
        
        # 场景参数
        if t < 5.0:
            scenario = "Phase 1: Ideal (Fast Transfer)"
            gf, fv, lv, av, pe, oe, bd, bv = 15.0, 0.1, 0.01, 0.02, 0.01, 0.05, 0.1, 0.05
        elif t < 10.0:
            p = (t - 5.0) / 5.0
            scenario = "Phase 2: Light Disturbance (Slow Align)"
            gf = 15.0 - p * 5.0; fv = 0.1 + p * 0.4
            lv = 0.01 + p * 0.08; av = 0.02 + p * 0.15
            pe = 0.01 + p * 0.03; oe = 0.05 + p * 0.15
            bd = 0.1 + p * 0.1; bv = 0.05 - p * 0.02
        elif t < 15.0:
            p = (t - 10.0) / 5.0
            scenario = "Phase 3: Medium Disturbance (Pause & Replan)"
            gf = 10.0 - p * 3.0; fv = 0.5 + p * 0.8
            lv = 0.09 + p * 0.08; av = 0.17 + p * 0.25
            pe = 0.04 + p * 0.05; oe = 0.2 + p * 0.2
            bd = 0.2 + p * 0.1; bv = 0.03 - p * 0.02
        elif t < 20.0:
            p = (t - 15.0) / 5.0
            scenario = "Phase 4: Severe Disturbance (Emergency Stop)"
            gf = 7.0 - p * 3.0; fv = 1.3 + p * 0.5
            lv = 0.17 + p * 0.1; av = 0.42 + p * 0.3
            pe = 0.09 + p * 0.05; oe = 0.4 + p * 0.3
            bd = 0.3 + p * 0.1; bv = 0.01 - p * 0.008
        else:
            p = (t - 20.0) / 10.0
            scenario = "Phase 5: Recovery (Online Replanning)"
            gf = 4.0 + p * 11.0; fv = 1.8 - p * 1.6
            lv = 0.27 - p * 0.24; av = 0.72 - p * 0.68
            pe = 0.14 - p * 0.12; oe = 0.7 - p * 0.6
            bd = 0.4 - p * 0.28; bv = 0.002 + p * 0.048
        
        noise = 0.03
        belief = controller.update(
            grip_force=gf + np.random.randn() * noise * 2,
            object_mass=0.5,
            force_variance=fv + np.random.randn() * noise,
            linear_vel=np.array([lv, lv*0.5, 0.0]) + np.random.randn(3) * noise * 0.1,
            angular_vel=np.array([0.0, 0.0, av]) + np.random.randn(3) * noise * 0.3,
            position_error=np.array([pe, pe*0.5, 0.0]) + np.random.randn(3) * noise * 0.01,
            orientation_error=oe + np.random.randn() * noise * 0.05,
            b_distance=bd + np.random.randn() * noise * 0.01,
            b_velocity=bv + np.random.randn() * noise * 0.005,
            current_time=t
        )
        
        timestamps.append(t)
        hcs_values.append(belief.hcs)
        strategies.append(belief.strategy.value)
        components['grasp'].append(belief.grasp_stability)
        components['velocity'].append(belief.object_velocity)
        components['alignment'].append(belief.alignment_error)
        components['b_readiness'].append(belief.b_readiness)
        
        # 每帧都生成图
        fig, axes = plt.subplots(3, 1, figsize=(16, 12))
        fig.suptitle('UAHP: Uncertainty-Aware Adaptive Handoff Policy', fontsize=18, fontweight='bold', y=0.98)
        
        # HCS曲线
        ax1 = axes[0]
        ax1.plot(timestamps, hcs_values, color='#e74c3c', linewidth=3, label='HCS')
        ax1.axhline(y=0.75, color='gray', linestyle='--', alpha=0.5)
        ax1.axhline(y=0.45, color='gray', linestyle=':', alpha=0.5)
        ax1.axhline(y=0.30, color='gray', linestyle='-.', alpha=0.5)
        ax1.axhspan(0.75, 1.0, alpha=0.1, color='green')
        ax1.axhspan(0.45, 0.75, alpha=0.1, color='blue')
        ax1.axhspan(0.30, 0.45, alpha=0.1, color='orange')
        ax1.axhspan(0.0, 0.30, alpha=0.1, color='red')
        
        cur_hcs = hcs_values[-1]
        cur_strat = strategies[-1]
        color = strategy_colors.get(cur_strat, 'gray')
        ax1.scatter([t], [cur_hcs], color=color, s=120, zorder=5)
        ax1.annotate(f'HCS={cur_hcs:.3f} → {cur_strat}', xy=(t, cur_hcs), xytext=(t+0.8, cur_hcs+0.08),
                    fontsize=11, fontweight='bold', arrowprops=dict(arrowstyle='->', color=color, lw=2))
        ax1.set_ylabel('HCS', fontsize=13)
        ax1.set_title('HCS Evolution Over Time', fontsize=14, fontweight='bold')
        ax1.set_ylim(0, 1); ax1.set_xlim(0, duration); ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper left', fontsize=10)
        
        # 分量
        ax2 = axes[1]
        ax2.plot(timestamps, components['grasp'], label='Grasp Stability', color='#2ecc71', linewidth=2)
        ax2.plot(timestamps, components['velocity'], label='Velocity Stability', color='#3498db', linewidth=2)
        ax2.plot(timestamps, components['alignment'], label='Alignment', color='#f39c12', linewidth=2)
        ax2.plot(timestamps, components['b_readiness'], label='B-Readiness', color='#9b59b6', linewidth=2)
        ax2.set_ylabel('Value', fontsize=13)
        ax2.set_title('HCS Components', fontsize=14, fontweight='bold')
        ax2.set_ylim(0, 1); ax2.set_xlim(0, duration); ax2.grid(True, alpha=0.3)
        ax2.legend(loc='upper left', fontsize=10)
        
        # 策略分布
        ax3 = axes[2]
        strat_counts = {}
        for s in strategies:
            strat_counts[s] = strat_counts.get(s, 0) + 1
        colors = [strategy_colors.get(s, 'gray') for s in strat_counts.keys()]
        bars = ax3.bar(strat_counts.keys(), strat_counts.values(), color=colors, alpha=0.8, width=0.6)
        for bar, cnt in zip(bars, strat_counts.values()):
            ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1, str(cnt), 
                    ha='center', va='bottom', fontsize=12, fontweight='bold')
        ax3.set_ylabel('Count', fontsize=13)
        ax3.set_title('Strategy Distribution', fontsize=14, fontweight='bold')
        ax3.grid(True, alpha=0.3, axis='y')
        
        fig.text(0.02, 0.02, f'Time: {t:.1f}s / {duration:.1f}s', fontsize=13, fontweight='bold')
        fig.text(0.98, 0.02, scenario, fontsize=13, fontweight='bold', ha='right', 
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        plt.savefig(f'uahp_frames_hq/frame_{frame:04d}.png', dpi=120, bbox_inches='tight')
        plt.close()
        
        if frame % 50 == 0:
            print(f"  进度: {frame}/{total_frames}")
    
    print(f"✅ 已生成 {len(os.listdir('uahp_frames_hq'))} 帧")
    
    # 用ffmpeg合成视频
    cmd = [
        'ffmpeg', '-y', '-framerate', str(fps),
        '-i', 'uahp_frames_hq/frame_%04d.png',
        '-vf', 'scale=1920:1080:force_original_aspect_ratio=decrease,pad=1920:1080:(ow-iw)/2:(oh-ih)/2',
        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '16',
        '-preset', 'slow',
        'uahp_demo_hq.mp4'
    ]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode == 0:
        size = os.path.getsize('uahp_demo_hq.mp4')
        print(f"✅ 视频已保存: uahp_demo_hq.mp4 ({size/1024/1024:.1f}MB)")
    else:
        print(f"❌ 失败: {result.stderr[-300:]}")


if __name__ == "__main__":
    generate_frames()