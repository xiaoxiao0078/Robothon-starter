"""
UAHP集成测试脚本
验证信念状态驱动的自适应交接控制
"""

import numpy as np
import time
import json
from typing import Dict, List

# 导入UAHP模块
from dual_arm.uahp import UAHPController, HandoffStrategy


def simulate_handoff_scenario(
    controller: UAHPController,
    scenario_name: str,
    duration: float = 5.0,
    dt: float = 0.1
) -> Dict:
    """
    模拟交接场景
    
    Args:
        controller: UAHP控制器
        scenario_name: 场景名称
        duration: 持续时间（秒）
        dt: 时间步长
    
    Returns:
        results: 模拟结果
    """
    print(f"\n🎬 模拟场景: {scenario_name}")
    print("-" * 40)
    
    # 记录
    beliefs = []
    actions = []
    timestamps = []
    
    # 模拟参数（根据场景调整）
    if scenario_name == "理想交接":
        # 稳定抓取，对齐良好
        grip_force = 15.0
        object_mass = 0.5
        force_variance = 0.1
        base_linear_vel = np.array([0.01, 0.0, 0.0])
        base_angular_vel = np.array([0.0, 0.0, 0.0])
        base_position_error = np.array([0.01, 0.0, 0.0])
        base_orientation_error = 0.05
        base_b_distance = 0.1
        base_b_velocity = 0.05
        
    elif scenario_name == "轻微扰动":
        # 有一些扰动
        grip_force = 12.0
        object_mass = 0.5
        force_variance = 0.3
        base_linear_vel = np.array([0.05, 0.02, 0.0])
        base_angular_vel = np.array([0.0, 0.0, 0.1])
        base_position_error = np.array([0.02, 0.01, 0.0])
        base_orientation_error = 0.1
        base_b_distance = 0.15
        base_b_velocity = 0.03
        
    elif scenario_name == "中等扰动":
        # 明显扰动
        grip_force = 8.0
        object_mass = 0.5
        force_variance = 0.5
        base_linear_vel = np.array([0.1, 0.05, 0.0])
        base_angular_vel = np.array([0.0, 0.0, 0.3])
        base_position_error = np.array([0.05, 0.02, 0.0])
        base_orientation_error = 0.2
        base_b_distance = 0.2
        base_b_velocity = 0.01
        
    elif scenario_name == "严重扰动":
        # 严重扰动（更极端）
        grip_force = 3.0  # 降低抓取力
        object_mass = 0.5
        force_variance = 2.0  # 增加力方差
        base_linear_vel = np.array([0.3, 0.15, 0.0])  # 增加速度
        base_angular_vel = np.array([0.0, 0.0, 0.8])  # 增加角速度
        base_position_error = np.array([0.15, 0.08, 0.0])  # 增加位置误差
        base_orientation_error = 0.8  # 增加姿态误差
        base_b_distance = 0.4  # 增加B臂距离
        base_b_velocity = 0.002  # 降低B臂速度
        
    elif scenario_name == "动态变化":
        # 动态变化的场景
        grip_force = 12.0
        object_mass = 0.5
        force_variance = 0.2
        base_linear_vel = np.array([0.03, 0.01, 0.0])
        base_angular_vel = np.array([0.0, 0.0, 0.05])
        base_position_error = np.array([0.015, 0.005, 0.0])
        base_orientation_error = 0.08
        base_b_distance = 0.12
        base_b_velocity = 0.04
    
    else:
        raise ValueError(f"未知场景: {scenario_name}")
    
    # 运行模拟
    for step in range(int(duration / dt)):
        t = step * dt
        
        # 添加随机扰动
        noise_scale = 0.1
        linear_vel = base_linear_vel + np.random.randn(3) * noise_scale * 0.1
        angular_vel = base_angular_vel + np.random.randn(3) * noise_scale * 0.5
        position_error = base_position_error + np.random.randn(3) * noise_scale * 0.01
        orientation_error = base_orientation_error + np.random.randn() * noise_scale * 0.1
        b_distance = base_b_distance + np.random.randn() * noise_scale * 0.02
        b_velocity = base_b_velocity + np.random.randn() * noise_scale * 0.01
        
        # 动态变化场景
        if scenario_name == "动态变化":
            # 模拟抓取力下降
            if t > 2.0:
                grip_force = max(5.0, grip_force - 0.5 * dt)
                force_variance = min(1.0, force_variance + 0.1 * dt)
            
            # 模拟对齐改善
            if t > 3.0:
                base_position_error = base_position_error * 0.95
                base_b_distance = base_b_distance * 0.95
        
        # 更新UAHP
        belief = controller.update(
            grip_force=grip_force,
            object_mass=object_mass,
            force_variance=force_variance,
            linear_vel=linear_vel,
            angular_vel=angular_vel,
            position_error=position_error,
            orientation_error=orientation_error,
            b_distance=b_distance,
            b_velocity=b_velocity,
            current_time=t
        )
        
        # 获取动作
        action = controller.get_action(belief)
        
        # 记录
        timestamps.append(t)
        beliefs.append({
            'hcs': belief.hcs,
            'grasp_stability': belief.grasp_stability,
            'velocity_stability': belief.object_velocity,
            'alignment': belief.alignment_error,
            'b_readiness': belief.b_readiness,
            'strategy': belief.strategy.value
        })
        actions.append(action)
        
        # 打印关键状态
        if step % 10 == 0:
            print(f"  t={t:.1f}s: HCS={belief.hcs:.3f}, 策略={belief.strategy.value}")
    
    # 统计
    stats = controller.get_stats()
    
    return {
        'scenario': scenario_name,
        'timestamps': timestamps,
        'beliefs': beliefs,
        'actions': actions,
        'stats': stats
    }


def run_comprehensive_test():
    """运行综合测试"""
    print("=" * 60)
    print("🚀 UAHP综合测试")
    print("=" * 60)
    
    # 创建控制器
    controller = UAHPController()
    
    # 测试场景
    scenarios = [
        "理想交接",
        "轻微扰动",
        "中等扰动",
        "严重扰动",
        "动态变化"
    ]
    
    all_results = {}
    
    for scenario in scenarios:
        # 重置控制器
        controller.reset()
        
        # 运行模拟
        results = simulate_handoff_scenario(
            controller,
            scenario,
            duration=5.0,
            dt=0.1
        )
        
        all_results[scenario] = results
        
        # 打印统计
        stats = results['stats']
        print(f"\n📊 统计:")
        print(f"  平均HCS: {stats['avg_hcs']:.3f}")
        print(f"  最小HCS: {stats['min_hcs']:.3f}")
        print(f"  快速交接: {stats['fast_transfers']}")
        print(f"  慢速对齐: {stats['slow_aligns']}")
        print(f"  暂停重规划: {stats['pause_replans']}")
        print(f"  紧急停止: {stats['emergency_stops']}")
        print(f"  恢复次数: {stats['recoveries']}")
    
    return all_results


def generate_comparison_report(results: Dict) -> str:
    """生成对比报告"""
    report = []
    report.append("# UAHP测试报告")
    report.append("\n## 场景对比\n")
    
    report.append("| 场景 | 平均HCS | 最小HCS | 快速交接 | 慢速对齐 | 暂停重规划 | 紧急停止 | 恢复次数 |")
    report.append("|------|---------|---------|----------|----------|------------|----------|----------|")
    
    for scenario, data in results.items():
        stats = data['stats']
        report.append(
            f"| {scenario} | {stats['avg_hcs']:.3f} | {stats['min_hcs']:.3f} | "
            f"{stats['fast_transfers']} | {stats['slow_aligns']} | "
            f"{stats['pause_replans']} | {stats['emergency_stops']} | "
            f"{stats['recoveries']} |"
        )
    
    report.append("\n## 关键发现\n")
    
    # 分析结果
    ideal_hcs = results['理想交接']['stats']['avg_hcs']
    disturbed_hcs = results['中等扰动']['stats']['avg_hcs']
    severe_hcs = results['严重扰动']['stats']['avg_hcs']
    
    report.append(f"1. **理想场景HCS**: {ideal_hcs:.3f} (接近1.0)")
    report.append(f"2. **中等扰动HCS**: {disturbed_hcs:.3f} (下降{(ideal_hcs-disturbed_hcs)/ideal_hcs*100:.1f}%)")
    report.append(f"3. **严重扰动HCS**: {severe_hcs:.3f} (下降{(ideal_hcs-severe_hcs)/ideal_hcs*100:.1f}%)")
    
    # 策略分布
    report.append("\n## 策略分布\n")
    
    for scenario, data in results.items():
        stats = data['stats']
        total = stats['total_decisions']
        report.append(f"**{scenario}**:")
        report.append(f"- 快速交接: {stats['fast_transfers']/total*100:.1f}%")
        report.append(f"- 慢速对齐: {stats['slow_aligns']/total*100:.1f}%")
        report.append(f"- 暂停重规划: {stats['pause_replans']/total*100:.1f}%")
        report.append(f"- 紧急停止: {stats['emergency_stops']/total*100:.1f}%")
        report.append("")
    
    return "\n".join(report)


def main():
    """主函数"""
    # 运行测试
    results = run_comprehensive_test()
    
    # 生成报告
    report = generate_comparison_report(results)
    
    # 保存报告
    with open('uahp_test_report.md', 'w', encoding='utf-8') as f:
        f.write(report)
    
    print("\n" + "=" * 60)
    print("✅ 测试完成！报告已保存到 uahp_test_report.md")
    print("=" * 60)
    
    # 打印关键指标
    print("\n📊 关键指标:")
    print(f"  理想场景HCS: {results['理想交接']['stats']['avg_hcs']:.3f}")
    print(f"  中等扰动HCS: {results['中等扰动']['stats']['avg_hcs']:.3f}")
    print(f"  严重扰动HCS: {results['严重扰动']['stats']['avg_hcs']:.3f}")
    print(f"  动态变化HCS: {results['动态变化']['stats']['avg_hcs']:.3f}")
    
    # 保存详细数据
    detailed_data = {}
    for scenario, data in results.items():
        detailed_data[scenario] = {
            'stats': data['stats'],
            'final_belief': data['beliefs'][-1] if data['beliefs'] else None
        }
    
    with open('uahp_detailed_results.json', 'w', encoding='utf-8') as f:
        json.dump(detailed_data, f, indent=2, ensure_ascii=False)
    
    print("\n💾 详细数据已保存到 uahp_detailed_results.json")
    
    return results


if __name__ == "__main__":
    main()