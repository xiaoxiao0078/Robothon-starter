"""
UAHP高质量视频渲染
30秒，1080p，展示完整UAHP工作流程
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
import json
from dual_arm.uahp import UAHPController, HandoffStrategy


def create_high_quality_video():
    """创建高质量UAHP演示视频"""
    controller = UAHPController()
    
    # 视频参数
    duration = 30.0  # 30秒
    fps = 30
    dt = 1.0 / fps
    frames = int(duration * dt)
    
    # 存储数据
    timestamps = []
    hcs_values = []
    strategies = []
    grasp_stability = []
    velocity_stability = []
    alignment = []
    b_readiness = []
    
    # 模拟场景：展示UAHP完整工作流程
    for frame in range(frames):
        t = frame * dt
        
        # 场景分段
        if t < 5.0:
            # 阶段1：理想场景（HCS > 0.75，fast_transfer）
            grip_force = 15.0
            force_variance = 0.1
            linear_vel = np.array([0.01, 0.005, 0.0])
            angular_vel = np.array([0.0, 0.0, 0.02])
            position_error = np.array([0.01, 0.005, 0.0])
            orientation_error = 0.05
            b_distance = 0.1
            b_velocity = 0.05
            scenario_label = "Ideal: HCS > 0.75 → Fast Transfer"
            
        elif t < 10.0:
            # 阶段2：轻微扰动开始（HCS下降到0.5-0.75，slow_align）
            progress = (t - 5.0) / 5.0
            grip_force = 15.0 - progress * 5.0
            force_variance = 0.1 + progress * 0.4
            linear_vel = np.array([0.01 + progress * 0.08, 0.005 + progress * 0.04, 0.0])
            angular_vel = np.array([0.0, 0.0, 0.02 + progress * 0.15])
            position_error = np.array([0.01 + progress * 0.03, 0.005 + progress * 0.015, 0.0])
            orientation_error = 0.05 + progress * 0.15
            b_distance = 0.1 + progress * 0.1
            b_velocity = 0.05 - progress * 0.02
            scenario_label = "Light Disturbance: HCS 0.5-0.75 → Slow Align"
            
        elif t < 15.0:
            # 阶段3：中等扰动（HCS 0.3-0.5，pause_replan）
            progress = (t - 10.0) / 5.0
            grip_force = 10.0 - progress * 3.0
            force_variance = 0.5 + progress * 0.8
            linear_vel = np.array([0.09 + progress * 0.08, 0.045 + progress * 0.04, 0.0])
            angular_vel = np.array([0.0, 0.0, 0.17 + progress * 0.25])
            position_error = np.array([0.04 + progress * 0.05, 0.02 + progress * 0.025, 0.0])
            orientation_error = 0.2 + progress * 0.2
            b_distance = 0.2 + progress * 0.1
            b_velocity = 0.03 - progress * 0.02
            scenario_label = "Medium Disturbance: HCS 0.3-0.5 → Pause & Replan"
            
        elif t < 20.0:
            # 阶段4：严重扰动（HCS < 0.3，emergency_stop）
            progress = (t - 15.0) / 5.0
            grip_force = 7.0 - progress * 3.0
            force_variance = 1.3 + progress * 0.5
            linear_vel = np.array([0.17 + progress * 0.1, 0.085 + progress * 0.05, 0.0])
            angular_vel = np.array([0.0, 0.0, 0.42 + progress * 0.3])
            position_error = np.array([0.09 + progress * 0.05, 0.045 + progress * 0.025, 0.0])
            orientation_error = 0.4 + progress * 0.3
            b_distance = 0.3 + progress * 0.1
            b_velocity = 0.01 - progress * 0.008
            scenario_label = "Severe Disturbance: HCS < 0.3 → Emergency Stop"
            
        else:
            # 阶段5：恢复过程（HCS从0.3恢复到0.8+）
            progress = (t - 20.0) / 10.0
            grip_force = 4.0 + progress * 11.0
            force_variance = 1.8 - progress * 1.6
            linear_vel = np.array([0.27 - progress * 0.24, 0.135 - progress * 0.12, 0.0])
            angular_vel = np.array([0.0, 0.0, 0.72 - progress * 0.68])
            position_error = np.array([0.14 - progress * 0.12, 0.07 - progress * 0.06, 0.0])
            orientation_error = 0.7 - progress * 0.6
            b_distance = 0.4 - progress * 0.28
            b_velocity = 0.002 + progress * 0.048
            scenario_label = "Recovery: HCS 0.3 → 0.8+ (Online Replanning)"
        
        # 添加随机扰动
        noise = 0.03
        belief = controller.update(
            grip_force=grip_force + np.random.randn() * noise * 2,
            object_mass=0.5,
            force_variance=force_variance + np.random.randn() * noise,
            linear_vel=linear_vel + np.random.randn(3) * noise * 0.1,
            angular_vel=angular_vel + np.random.randn(3) * noise * 0.3,
            position_error=position_error + np.random.randn(3) * noise * 0.01,
            orientation_error=orientation_error + np.random.randn() * noise * 0.05,
            b_distance=b_distance + np.random.randn() * noise * 0.01,
            b_velocity=b_velocity + np.random.randn() * noise * 0.005,
            current_time=t
        )
        
        timestamps.append(t)
        hcs_values.append(belief.hcs)
        strategies.append(belief.strategy.value)
        grasp_stability.append(belief.grasp_stability)
        velocity_stability.append(belief.object_velocity)
        alignment.append(belief.alignment_error)
        b_readiness.append(belief.b_readiness)
    
    # 创建高质量图表
    fig, axes = plt.subplots(3, 1, figsize=(16, 12))
    fig.suptitle('UAHP: Uncertainty-Aware Adaptive Handoff Policy', 
                 fontsize=18, fontweight='bold', y=0.98)
    
    # 颜色映射
    strategy_colors = {
        'fast_transfer': '#2ecc71',
        'slow_align': '#3498db',
        'pause_replan': '#f39c12',
        'emergency_stop': '#e74c3c'
    }
    
    def animate(frame):
        # 清除所有子图
        for ax in axes:
            ax.clear()
        
        t = frame * dt
        
        # 上图：HCS曲线
        ax1 = axes[0]
        ax1.plot(timestamps[:frame+1], hcs_values[:frame+1], color='#e74c3c', linewidth=3, label='HCS')
        ax1.axhline(y=0.75, color='gray', linestyle='--', alpha=0.5, label='Fast Transfer (0.75)')
        ax1.axhline(y=0.45, color='gray', linestyle=':', alpha=0.5, label='Slow Align (0.45)')
        ax1.axhline(y=0.30, color='gray', linestyle='-.', alpha=0.5, label='Pause/Replan (0.30)')
        
        # 填充区域
        ax1.axhspan(0.75, 1.0, alpha=0.1, color='green', label='Fast Transfer Zone')
        ax1.axhspan(0.45, 0.75, alpha=0.1, color='blue', label='Slow Align Zone')
        ax1.axhspan(0.30, 0.45, alpha=0.1, color='orange', label='Pause/Replan Zone')
        ax1.axhspan(0.0, 0.30, alpha=0.1, color='red', label='Emergency Stop Zone')
        
        # 标记当前点
        if frame > 0:
            current_hcs = hcs_values[frame]
            current_strategy = strategies[frame]
            color = strategy_colors.get(current_strategy, 'gray')
            ax1.scatter([t], [current_hcs], color=color, s=100, zorder=5)
            ax1.annotate(f'HCS={current_hcs:.3f}\n{current_strategy}', 
                        xy=(t, current_hcs), xytext=(t+0.5, current_hcs+0.05),
                        fontsize=10, ha='left',
                        arrowprops=dict(arrowstyle='->', color=color))
        
        ax1.set_ylabel('Handoff Confidence Score (HCS)', fontsize=12)
        ax1.set_title('HCS Evolution Over Time', fontsize=14, fontweight='bold')
        ax1.set_ylim(0.0, 1.0)
        ax1.legend(loc='upper left', fontsize=9)
        ax1.grid(True, alpha=0.3)
        ax1.set_xlim(0, duration)
        
        # 中图：各分量
        ax2 = axes[1]
        ax2.plot(timestamps[:frame+1], grasp_stability[:frame+1], label='Grasp Stability', color='#2ecc71', linewidth=2)
        ax2.plot(timestamps[:frame+1], velocity_stability[:frame+1], label='Velocity Stability', color='#3498db', linewidth=2)
        ax2.plot(timestamps[:frame+1], alignment[:frame+1], label='Alignment', color='#f39c12', linewidth=2)
        ax2.plot(timestamps[:frame+1], b_readiness[:frame+1], label='B-Readiness', color='#9b59b6', linewidth=2)
        
        ax2.set_ylabel('Component Value', fontsize=12)
        ax2.set_title('HCS Components (Grasp + Velocity + Alignment + B-Readiness)', fontsize=14, fontweight='bold')
        ax2.set_ylim(0.0, 1.0)
        ax2.legend(loc='upper left', fontsize=9)
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim(0, duration)
        
        # 下图：策略分布（累计）
        ax3 = axes[2]
        if frame > 0:
            strategy_counts = {}
            for s in strategies[:frame+1]:
                strategy_counts[s] = strategy_counts.get(s, 0) + 1
            
            colors = [strategy_colors.get(s, 'gray') for s in strategy_counts.keys()]
            bars = ax3.bar(strategy_counts.keys(), strategy_counts.values(), color=colors, alpha=0.8)
            
            # 添加数值标签
            for bar, count in zip(bars, strategy_counts.values()):
                ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                        str(count), ha='center', va='bottom', fontsize=11, fontweight='bold')
        
        ax3.set_ylabel('Strategy Count', fontsize=12)
        ax3.set_title('Strategy Distribution (Cumulative)', fontsize=14, fontweight='bold')
        ax3.grid(True, alpha=0.3, axis='y')
        
        # 添加时间戳
        fig.text(0.02, 0.02, f'Time: {t:.1f}s / {duration:.1f}s', fontsize=12, 
                fontweight='bold', ha='left')
        
        # 添加场景说明
        if t < 5.0:
            scenario_text = "Phase 1: Ideal Scenario"
        elif t < 10.0:
            scenario_text = "Phase 2: Light Disturbance"
        elif t < 15.0:
            scenario_text = "Phase 3: Medium Disturbance"
        elif t < 20.0:
            scenario_text = "Phase 4: Severe Disturbance"
        else:
            scenario_text = "Phase 5: Recovery (Online Replanning)"
        
        fig.text(0.98, 0.02, scenario_text, fontsize=12, 
                fontweight='bold', ha='right')
        
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # 创建动画
    anim = FuncAnimation(fig, animate, frames=frames, interval=1000//fps, repeat=True)
    
    # 保存为高质量MP4
    anim.save('uahp_demo_hq.mp4', writer='ffmpeg', fps=fps, dpi=150, 
              bitrate=5000, codec='libx264')
    plt.close()
    
    print("✅ 高质量UAHP动画已保存: uahp_demo_hq.mp4")
    
    # 保存静态图（最后一帧）
    fig, axes = plt.subplots(3, 1, figsize=(16, 12))
    fig.suptitle('UAHP: Uncertainty-Aware Adaptive Handoff Policy', 
                 fontsize=18, fontweight='bold', y=0.98)
    
    # 上图：HCS曲线
    ax1 = axes[0]
    ax1.plot(timestamps, hcs_values, color='#e74c3c', linewidth=3, label='HCS')
    ax1.axhline(y=0.75, color='gray', linestyle='--', alpha=0.5, label='Fast Transfer (0.75)')
    ax1.axhline(y=0.45, color='gray', linestyle=':', alpha=0.5, label='Slow Align (0.45)')
    ax1.axhline(y=0.30, color='gray', linestyle='-.', alpha=0.5, label='Pause/Replan (0.30)')
    
    ax1.axhspan(0.75, 1.0, alpha=0.1, color='green')
    ax1.axhspan(0.45, 0.75, alpha=0.1, color='blue')
    ax1.axhspan(0.30, 0.45, alpha=0.1, color='orange')
    ax1.axhspan(0.0, 0.30, alpha=0.1, color='red')
    
    ax1.set_ylabel('Handoff Confidence Score (HCS)', fontsize=12)
    ax1.set_title('HCS Evolution Over Time', fontsize=14, fontweight='bold')
    ax1.set_ylim(0.0, 1.0)
    ax1.legend(loc='upper left', fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(0, duration)
    
    # 中图：各分量
    ax2 = axes[1]
    ax2.plot(timestamps, grasp_stability, label='Grasp Stability', color='#2ecc71', linewidth=2)
    ax2.plot(timestamps, velocity_stability, label='Velocity Stability', color='#3498db', linewidth=2)
    ax2.plot(timestamps, alignment, label='Alignment', color='#f39c12', linewidth=2)
    ax2.plot(timestamps, b_readiness, label='B-Readiness', color='#9b59b6', linewidth=2)
    
    ax2.set_ylabel('Component Value', fontsize=12)
    ax2.set_title('HCS Components', fontsize=14, fontweight='bold')
    ax2.set_ylim(0.0, 1.0)
    ax2.legend(loc='upper left', fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(0, duration)
    
    # 下图：策略分布
    ax3 = axes[2]
    strategy_counts = {}
    for s in strategies:
        strategy_counts[s] = strategy_counts.get(s, 0) + 1
    
    colors = [strategy_colors.get(s, 'gray') for s in strategy_counts.keys()]
    bars = ax3.bar(strategy_counts.keys(), strategy_counts.values(), color=colors, alpha=0.8)
    
    for bar, count in zip(bars, strategy_counts.values()):
        ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                str(count), ha='center', va='bottom', fontsize=11, fontweight='bold')
    
    ax3.set_ylabel('Strategy Count', fontsize=12)
    ax3.set_title('Strategy Distribution (Total)', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('uahp_static_hq.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    print("✅ 高质量静态图已保存: uahp_static_hq.png")
    
    # 保存数据
    data = {
        'duration': duration,
        'fps': fps,
        'frames': frames,
        'timestamps': timestamps,
        'hcs_values': hcs_values,
        'strategies': strategies,
        'components': {
            'grasp_stability': grasp_stability,
            'velocity_stability': velocity_stability,
            'alignment': alignment,
            'b_readiness': b_readiness
        },
        'strategy_distribution': strategy_counts,
        'phases': [
            {'start': 0, 'end': 5, 'label': 'Ideal Scenario'},
            {'start': 5, 'end': 10, 'label': 'Light Disturbance'},
            {'start': 10, 'end': 15, 'label': 'Medium Disturbance'},
            {'start': 15, 'end': 20, 'label': 'Severe Disturbance'},
            {'start': 20, 'end': 30, 'label': 'Recovery'}
        ]
    }
    
    with open('uahp_hq_data.json', 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    
    print("✅ 高质量数据已保存: uahp_hq_data.json")


if __name__ == "__main__":
    create_high_quality_video()