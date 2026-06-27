"""
UAHP视频渲染脚本
展示HCS变化和策略切换
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
import json
from dual_arm.uahp import UAHPController, HandoffStrategy


def create_uahp_animation():
    """创建UAHP动画"""
    controller = UAHPController()
    
    # 模拟数据
    duration = 10.0  # 10秒
    dt = 0.1
    frames = int(duration / dt)
    
    # 存储数据
    timestamps = []
    hcs_values = []
    strategies = []
    grasp_stability = []
    velocity_stability = []
    alignment = []
    b_readiness = []
    
    # 模拟场景：从理想到扰动再到恢复
    for frame in range(frames):
        t = frame * dt
        
        # 动态变化的场景
        if t < 3.0:
            # 理想场景
            grip_force = 15.0
            force_variance = 0.1
            linear_vel = np.array([0.01, 0.0, 0.0])
            angular_vel = np.array([0.0, 0.0, 0.0])
            position_error = np.array([0.01, 0.0, 0.0])
            orientation_error = 0.05
            b_distance = 0.1
            b_velocity = 0.05
        elif t < 5.0:
            # 扰动开始
            progress = (t - 3.0) / 2.0
            grip_force = 15.0 - progress * 10.0
            force_variance = 0.1 + progress * 1.5
            linear_vel = np.array([0.01 + progress * 0.2, 0.0 + progress * 0.1, 0.0])
            angular_vel = np.array([0.0, 0.0, 0.0 + progress * 0.7])
            position_error = np.array([0.01 + progress * 0.12, 0.0 + progress * 0.06, 0.0])
            orientation_error = 0.05 + progress * 0.6
            b_distance = 0.1 + progress * 0.25
            b_velocity = 0.05 - progress * 0.045
        elif t < 7.0:
            # 严重扰动
            grip_force = 5.0
            force_variance = 1.6
            linear_vel = np.array([0.21, 0.1, 0.0])
            angular_vel = np.array([0.0, 0.0, 0.7])
            position_error = np.array([0.13, 0.06, 0.0])
            orientation_error = 0.65
            b_distance = 0.35
            b_velocity = 0.005
        else:
            # 恢复过程
            progress = (t - 7.0) / 3.0
            grip_force = 5.0 + progress * 10.0
            force_variance = 1.6 - progress * 1.4
            linear_vel = np.array([0.21 - progress * 0.19, 0.1 - progress * 0.09, 0.0])
            angular_vel = np.array([0.0, 0.0, 0.7 - progress * 0.65])
            position_error = np.array([0.13 - progress * 0.11, 0.06 - progress * 0.05, 0.0])
            orientation_error = 0.65 - progress * 0.55
            b_distance = 0.35 - progress * 0.23
            b_velocity = 0.005 + progress * 0.04
        
        # 添加随机扰动
        noise = 0.05
        belief = controller.update(
            grip_force=grip_force + np.random.randn() * noise,
            object_mass=0.5,
            force_variance=force_variance + np.random.randn() * noise * 0.5,
            linear_vel=linear_vel + np.random.randn(3) * noise * 0.1,
            angular_vel=angular_vel + np.random.randn(3) * noise * 0.5,
            position_error=position_error + np.random.randn(3) * noise * 0.01,
            orientation_error=orientation_error + np.random.randn() * noise * 0.1,
            b_distance=b_distance + np.random.randn() * noise * 0.02,
            b_velocity=b_velocity + np.random.randn() * noise * 0.01,
            current_time=t
        )
        
        timestamps.append(t)
        hcs_values.append(belief.hcs)
        strategies.append(belief.strategy.value)
        grasp_stability.append(belief.grasp_stability)
        velocity_stability.append(belief.object_velocity)
        alignment.append(belief.alignment_error)
        b_readiness.append(belief.b_readiness)
    
    # 创建动画
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
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
        
        # 上图：HCS曲线
        ax1 = axes[0]
        ax1.plot(timestamps[:frame+1], hcs_values[:frame+1], color='#e74c3c', linewidth=3, label='HCS')
        ax1.axhline(y=0.75, color='gray', linestyle='--', alpha=0.5)
        ax1.axhline(y=0.45, color='gray', linestyle=':', alpha=0.5)
        ax1.axhline(y=0.30, color='gray', linestyle='-.', alpha=0.5)
        
        # 填充区域
        ax1.axhspan(0.75, 1.0, alpha=0.1, color='green')
        ax1.axhspan(0.45, 0.75, alpha=0.1, color='blue')
        ax1.axhspan(0.30, 0.45, alpha=0.1, color='orange')
        ax1.axhspan(0.0, 0.30, alpha=0.1, color='red')
        
        ax1.set_ylabel('HCS', fontsize=12)
        ax1.set_title('UAHP: Handoff Confidence Score', fontsize=14, fontweight='bold')
        ax1.set_ylim(0.0, 1.0)
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)
        
        # 中图：各分量
        ax2 = axes[1]
        ax2.plot(timestamps[:frame+1], grasp_stability[:frame+1], label='Grasp Stability', color='#2ecc71', linewidth=2)
        ax2.plot(timestamps[:frame+1], velocity_stability[:frame+1], label='Velocity Stability', color='#3498db', linewidth=2)
        ax2.plot(timestamps[:frame+1], alignment[:frame+1], label='Alignment', color='#f39c12', linewidth=2)
        ax2.plot(timestamps[:frame+1], b_readiness[:frame+1], label='B-Readiness', color='#9b59b6', linewidth=2)
        
        ax2.set_ylabel('Component Value', fontsize=12)
        ax2.set_title('HCS Components', fontsize=14, fontweight='bold')
        ax2.set_ylim(0.0, 1.0)
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)
        
        # 下图：策略分布
        ax3 = axes[2]
        if frame > 0:
            current_strategy = strategies[frame]
            color = strategy_colors.get(current_strategy, 'gray')
            ax3.bar(['Current Strategy'], [1], color=color, alpha=0.8)
            ax3.set_ylabel('Strategy', fontsize=12)
            ax3.set_title(f'Current Strategy: {current_strategy}', fontsize=14, fontweight='bold')
            ax3.set_ylim(0, 1.2)
            
            # 添加策略说明
            strategy_text = {
                'fast_transfer': 'Fast Transfer (HCS > 0.75)',
                'slow_align': 'Slow Align (0.45 < HCS ≤ 0.75)',
                'pause_replan': 'Pause & Replan (0.30 < HCS ≤ 0.45)',
                'emergency_stop': 'Emergency Stop (HCS ≤ 0.30)'
            }
            ax3.text(0.5, 0.5, strategy_text.get(current_strategy, ''), 
                    ha='center', va='center', fontsize=12, transform=ax3.transAxes)
        
        # 设置x轴范围
        for ax in axes:
            ax.set_xlim(0, duration)
        
        axes[2].set_xlabel('Time (seconds)', fontsize=12)
        
        plt.tight_layout()
    
    # 创建动画
    anim = FuncAnimation(fig, animate, frames=frames, interval=100, repeat=True)
    
    # 保存为MP4
    anim.save('uahp_demo.mp4', writer='ffmpeg', fps=10, dpi=100)
    plt.close()
    
    print("✅ UAHP动画已保存: uahp_demo.mp4")
    
    # 保存静态图
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # 上图：HCS曲线
    ax1 = axes[0]
    ax1.plot(timestamps, hcs_values, color='#e74c3c', linewidth=3, label='HCS')
    ax1.axhline(y=0.75, color='gray', linestyle='--', alpha=0.5)
    ax1.axhline(y=0.45, color='gray', linestyle=':', alpha=0.5)
    ax1.axhline(y=0.30, color='gray', linestyle='-.', alpha=0.5)
    
    # 填充区域
    ax1.axhspan(0.75, 1.0, alpha=0.1, color='green')
    ax1.axhspan(0.45, 0.75, alpha=0.1, color='blue')
    ax1.axhspan(0.30, 0.45, alpha=0.1, color='orange')
    ax1.axhspan(0.0, 0.30, alpha=0.1, color='red')
    
    ax1.set_ylabel('HCS', fontsize=12)
    ax1.set_title('UAHP: Handoff Confidence Score', fontsize=14, fontweight='bold')
    ax1.set_ylim(0.0, 1.0)
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    
    # 中图：各分量
    ax2 = axes[1]
    ax2.plot(timestamps, grasp_stability, label='Grasp Stability', color='#2ecc71', linewidth=2)
    ax2.plot(timestamps, velocity_stability, label='Velocity Stability', color='#3498db', linewidth=2)
    ax2.plot(timestamps, alignment, label='Alignment', color='#f39c12', linewidth=2)
    ax2.plot(timestamps, b_readiness, label='B-Readiness', color='#9b59b6', linewidth=2)
    
    ax2.set_ylabel('Component Value', fontsize=12)
    ax2.set_title('HCS Components', fontsize=14, fontweight='bold')
    ax2.set_ylim(0.0, 1.0)
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # 下图：策略分布
    ax3 = axes[2]
    strategy_counts = {}
    for s in strategies:
        strategy_counts[s] = strategy_counts.get(s, 0) + 1
    
    colors = [strategy_colors.get(s, 'gray') for s in strategy_counts.keys()]
    ax3.bar(strategy_counts.keys(), strategy_counts.values(), color=colors, alpha=0.8)
    ax3.set_ylabel('Count', fontsize=12)
    ax3.set_title('Strategy Distribution', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.3, axis='y')
    
    # 设置x轴范围
    for ax in axes[:2]:
        ax.set_xlim(0, duration)
    
    axes[2].set_xlabel('Strategy', fontsize=12)
    
    plt.tight_layout()
    plt.savefig('uahp_static.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    print("✅ UAHP静态图已保存: uahp_static.png")
    
    # 保存数据
    data = {
        'timestamps': timestamps,
        'hcs_values': hcs_values,
        'strategies': strategies,
        'components': {
            'grasp_stability': grasp_stability,
            'velocity_stability': velocity_stability,
            'alignment': alignment,
            'b_readiness': b_readiness
        },
        'strategy_distribution': strategy_counts
    }
    
    with open('uahp_animation_data.json', 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    
    print("✅ 动画数据已保存: uahp_animation_data.json")


if __name__ == "__main__":
    create_uahp_animation()