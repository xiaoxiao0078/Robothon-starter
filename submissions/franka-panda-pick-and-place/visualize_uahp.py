"""
UAHP可视化脚本
生成HCS曲线和策略分布图
"""

import numpy as np
import matplotlib.pyplot as plt
import json
from dual_arm.uahp import UAHPController, HandoffStrategy


def run_scenario(controller, scenario_name, duration=5.0, dt=0.1):
    """运行单个场景并收集数据"""
    # 场景参数
    params = {
        "理想交接": {
            'grip_force': 15.0, 'object_mass': 0.5, 'force_variance': 0.1,
            'base_linear_vel': np.array([0.01, 0.0, 0.0]),
            'base_angular_vel': np.array([0.0, 0.0, 0.0]),
            'base_position_error': np.array([0.01, 0.0, 0.0]),
            'base_orientation_error': 0.05,
            'base_b_distance': 0.1, 'base_b_velocity': 0.05
        },
        "轻微扰动": {
            'grip_force': 12.0, 'object_mass': 0.5, 'force_variance': 0.3,
            'base_linear_vel': np.array([0.05, 0.02, 0.0]),
            'base_angular_vel': np.array([0.0, 0.0, 0.1]),
            'base_position_error': np.array([0.02, 0.01, 0.0]),
            'base_orientation_error': 0.1,
            'base_b_distance': 0.15, 'base_b_velocity': 0.03
        },
        "中等扰动": {
            'grip_force': 8.0, 'object_mass': 0.5, 'force_variance': 0.5,
            'base_linear_vel': np.array([0.1, 0.05, 0.0]),
            'base_angular_vel': np.array([0.0, 0.0, 0.3]),
            'base_position_error': np.array([0.05, 0.02, 0.0]),
            'base_orientation_error': 0.2,
            'base_b_distance': 0.2, 'base_b_velocity': 0.01
        },
        "严重扰动": {
            'grip_force': 5.0, 'object_mass': 0.5, 'force_variance': 1.0,
            'base_linear_vel': np.array([0.2, 0.1, 0.0]),
            'base_angular_vel': np.array([0.0, 0.0, 0.5]),
            'base_position_error': np.array([0.1, 0.05, 0.0]),
            'base_orientation_error': 0.5,
            'base_b_distance': 0.3, 'base_b_velocity': 0.005
        }
    }
    
    p = params[scenario_name]
    
    timestamps = []
    hcs_values = []
    strategies = []
    
    for step in range(int(duration / dt)):
        t = step * dt
        
        # 添加随机扰动
        noise = 0.1
        belief = controller.update(
            grip_force=p['grip_force'],
            object_mass=p['object_mass'],
            force_variance=p['force_variance'],
            linear_vel=p['base_linear_vel'] + np.random.randn(3) * noise * 0.1,
            angular_vel=p['base_angular_vel'] + np.random.randn(3) * noise * 0.5,
            position_error=p['base_position_error'] + np.random.randn(3) * noise * 0.01,
            orientation_error=p['base_orientation_error'] + np.random.randn() * noise * 0.1,
            b_distance=p['base_b_distance'] + np.random.randn() * noise * 0.02,
            b_velocity=p['base_b_velocity'] + np.random.randn() * noise * 0.01,
            current_time=t
        )
        
        timestamps.append(t)
        hcs_values.append(belief.hcs)
        strategies.append(belief.strategy.value)
    
    return timestamps, hcs_values, strategies


def plot_hcs_curves():
    """绘制HCS曲线图"""
    controller = UAHPController()
    
    scenarios = ["理想交接", "轻微扰动", "中等扰动", "严重扰动"]
    colors = ['#2ecc71', '#3498db', '#f39c12', '#e74c3c']
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    for scenario, color in zip(scenarios, colors):
        controller.reset()
        timestamps, hcs_values, _ = run_scenario(controller, scenario)
        ax.plot(timestamps, hcs_values, label=scenario, color=color, linewidth=2)
    
    # 添加阈值线
    ax.axhline(y=0.8, color='gray', linestyle='--', alpha=0.5, label='Fast Transfer阈值 (0.8)')
    ax.axhline(y=0.5, color='gray', linestyle=':', alpha=0.5, label='Slow Align阈值 (0.5)')
    
    # 填充区域
    ax.axhspan(0.8, 1.0, alpha=0.1, color='green', label='Fast Transfer区域')
    ax.axhspan(0.5, 0.8, alpha=0.1, color='blue', label='Slow Align区域')
    ax.axhspan(0.0, 0.5, alpha=0.1, color='red', label='Pause/Replan区域')
    
    ax.set_xlabel('时间 (秒)', fontsize=12)
    ax.set_ylabel('Handoff Confidence Score (HCS)', fontsize=12)
    ax.set_title('UAHP: 不同扰动场景下的HCS变化曲线', fontsize=14, fontweight='bold')
    ax.legend(loc='lower left', fontsize=10)
    ax.set_ylim(0.0, 1.0)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('uahp_hcs_curves.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    print("✅ HCS曲线图已保存: uahp_hcs_curves.png")


def plot_strategy_distribution():
    """绘制策略分布图"""
    controller = UAHPController()
    
    scenarios = ["理想交接", "轻微扰动", "中等扰动", "严重扰动"]
    
    # 收集数据
    strategy_counts = {s: {} for s in scenarios}
    
    for scenario in scenarios:
        controller.reset()
        _, _, strategies = run_scenario(controller, scenario)
        
        for strategy in strategies:
            if strategy not in strategy_counts[scenario]:
                strategy_counts[scenario][strategy] = 0
            strategy_counts[scenario][strategy] += 1
    
    # 准备绘图数据
    strategy_labels = ['fast_transfer', 'slow_align', 'pause_replan', 'emergency_stop']
    strategy_colors = ['#2ecc71', '#3498db', '#f39c12', '#e74c3c']
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    x = np.arange(len(scenarios))
    width = 0.2
    
    for i, (strategy, color) in enumerate(zip(strategy_labels, strategy_colors)):
        values = [strategy_counts[s].get(strategy, 0) for s in scenarios]
        ax.bar(x + i * width, values, width, label=strategy, color=color)
    
    ax.set_xlabel('场景', fontsize=12)
    ax.set_ylabel('决策次数', fontsize=12)
    ax.set_title('UAHP: 不同场景下的策略分布', fontsize=14, fontweight='bold')
    ax.set_xticks(x + width * 1.5)
    ax.set_xticklabels(scenarios)
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig('uahp_strategy_distribution.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    print("✅ 策略分布图已保存: uahp_strategy_distribution.png")


def plot_hcs_components():
    """绘制HCS各分量图"""
    controller = UAHPController()
    
    # 运行动态变化场景
    timestamps = []
    components = {
        'grasp_stability': [],
        'velocity_stability': [],
        'alignment': [],
        'b_readiness': [],
        'hcs': []
    }
    
    duration = 5.0
    dt = 0.1
    
    for step in range(int(duration / dt)):
        t = step * dt
        
        # 模拟抓取力下降
        grip_force = 12.0
        force_variance = 0.2
        if t > 2.0:
            grip_force = max(5.0, grip_force - 0.5 * dt)
            force_variance = min(1.0, force_variance + 0.1 * dt)
        
        belief = controller.update(
            grip_force=grip_force,
            object_mass=0.5,
            force_variance=force_variance,
            linear_vel=np.array([0.03, 0.01, 0.0]) + np.random.randn(3) * 0.01,
            angular_vel=np.array([0.0, 0.0, 0.05]) + np.random.randn(3) * 0.05,
            position_error=np.array([0.015, 0.005, 0.0]) + np.random.randn(3) * 0.001,
            orientation_error=0.08 + np.random.randn() * 0.01,
            b_distance=0.12 + np.random.randn() * 0.002,
            b_velocity=0.04 + np.random.randn() * 0.001,
            current_time=t
        )
        
        timestamps.append(t)
        components['grasp_stability'].append(belief.grasp_stability)
        components['velocity_stability'].append(belief.object_velocity)
        components['alignment'].append(belief.alignment_error)
        components['b_readiness'].append(belief.b_readiness)
        components['hcs'].append(belief.hcs)
    
    # 绘图
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # 上图：各分量
    ax1 = axes[0]
    ax1.plot(timestamps, components['grasp_stability'], label='抓取稳定性', color='#2ecc71', linewidth=2)
    ax1.plot(timestamps, components['velocity_stability'], label='速度稳定性', color='#3498db', linewidth=2)
    ax1.plot(timestamps, components['alignment'], label='对齐度', color='#f39c12', linewidth=2)
    ax1.plot(timestamps, components['b_readiness'], label='B臂准备度', color='#9b59b6', linewidth=2)
    
    ax1.set_ylabel('分量值', fontsize=12)
    ax1.set_title('UAHP: HCS各分量变化（动态场景）', fontsize=14, fontweight='bold')
    ax1.legend(loc='lower left')
    ax1.set_ylim(0.0, 1.0)
    ax1.grid(True, alpha=0.3)
    
    # 下图：综合HCS
    ax2 = axes[1]
    ax2.plot(timestamps, components['hcs'], label='HCS (综合)', color='#e74c3c', linewidth=3)
    ax2.axhline(y=0.8, color='gray', linestyle='--', alpha=0.5)
    ax2.axhline(y=0.5, color='gray', linestyle=':', alpha=0.5)
    ax2.axhspan(0.8, 1.0, alpha=0.1, color='green')
    ax2.axhspan(0.5, 0.8, alpha=0.1, color='blue')
    ax2.axhspan(0.0, 0.5, alpha=0.1, color='red')
    
    ax2.set_xlabel('时间 (秒)', fontsize=12)
    ax2.set_ylabel('HCS值', fontsize=12)
    ax2.legend()
    ax2.set_ylim(0.0, 1.0)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('uahp_components.png', dpi=150, bbox_inches='tight')
    plt.close()
    
    print("✅ HCS分量图已保存: uahp_components.png")


def main():
    """主函数"""
    print("=" * 50)
    print("📊 生成UAHP可视化图表")
    print("=" * 50)
    
    plot_hcs_curves()
    plot_strategy_distribution()
    plot_hcs_components()
    
    print("\n✅ 所有图表生成完成！")


if __name__ == "__main__":
    main()