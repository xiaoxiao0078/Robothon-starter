"""
UAHP: Uncertainty-Aware Adaptive Handoff Policy
核心创新：用信念状态驱动的策略控制替代确定性触发

三层架构：
1. Belief State (HCS) - 信念状态计算
2. Adaptive Decision Policy - 自适应决策策略
3. Online Recovery Replanning - 在线恢复重规划
"""

import numpy as np
from typing import Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class HandoffStrategy(Enum):
    """交接策略枚举"""
    FAST_TRANSFER = "fast_transfer"      # HCS > 0.8
    SLOW_ALIGN = "slow_align"            # 0.5 < HCS <= 0.8
    PAUSE_REPLAN = "pause_replan"        # HCS <= 0.5
    EMERGENCY_STOP = "emergency_stop"    # HCS < 0.2


@dataclass
class BeliefState:
    """信念状态数据结构"""
    hcs: float                          # Handoff Confidence Score [0, 1]
    grasp_stability: float              # 抓取稳定性 [0, 1]
    object_velocity: float              # 物体速度稳定性 [0, 1]
    alignment_error: float              # 对齐误差 [0, 1]
    b_readiness: float                  # B臂准备度 [0, 1]
    strategy: HandoffStrategy           # 当前策略
    timestamp: float                    # 时间戳


class HCSComputer:
    """
    Handoff Confidence Score 计算器
    
    综合多个因素计算交接置信度：
    - 物体抓取稳定性
    - 物体速度稳定性
    - B臂准备度
    - 对齐误差
    """
    
    def __init__(self, weights: Optional[Dict[str, float]] = None):
        """
        初始化HCS计算器
        
        Args:
            weights: 各因素权重，如果为None使用默认权重
        """
        # 默认权重（可调优）
        self.weights = weights or {
            'grasp_stability': 0.30,      # 抓取稳定性
            'object_velocity': 0.25,      # 速度稳定性
            'alignment_error': 0.25,      # 对齐误差
            'b_readiness': 0.20           # B臂准备度
        }
        
        # 历史记录（用于平滑）
        self.history_size = 3  # 减少历史窗口，提高响应速度
        self.hcs_history = []
        
        # 阈值配置（调整为更敏感）
        self.thresholds = {
            'fast_transfer': 0.75,        # 降低阈值，更容易触发
            'slow_align': 0.45,           # 降低阈值
            'pause_replan': 0.30,         # 调整阈值
            'emergency_stop': 0.25        # 调整阈值，让严重扰动触发
        }
    
    def compute_grasp_stability(
        self,
        grip_force: float,
        object_mass: float,
        force_variance: float
    ) -> float:
        """
        计算抓取稳定性
        
        Args:
            grip_force: 当前夹持力 (N)
            object_mass: 物体质量 (kg)
            force_variance: 力的方差（稳定性指标）
        
        Returns:
            stability: [0, 1] 越高越稳定
        """
        # 理论最小力 = 质量 * 重力 * 安全系数
        min_force = object_mass * 9.81 * 1.2  # 降低安全系数
        
        # 力充足度（使用更宽松的映射）
        force_adequacy = min(grip_force / min_force, 1.0)
        # 增益映射：0.6以上就算好
        force_adequacy = min(force_adequacy * 1.2, 1.0)
        
        # 力稳定性（方差越小越稳定，降低敏感度）
        force_stability = np.exp(-force_variance * 5)
        
        # 综合得分
        stability = 0.6 * force_adequacy + 0.4 * force_stability
        
        return np.clip(stability, 0.0, 1.0)
    
    def compute_velocity_stability(
        self,
        linear_vel: np.ndarray,
        angular_vel: np.ndarray,
        velocity_threshold: float = 0.01
    ) -> float:
        """
        计算物体速度稳定性
        
        Args:
            linear_vel: 线速度 (m/s)
            angular_vel: 角速度 (rad/s)
            velocity_threshold: 速度阈值
        
        Returns:
            stability: [0, 1] 越高越稳定（速度越小）
        """
        # 线速度稳定性（放宽阈值，0.05m/s以内都算稳定）
        linear_speed = np.linalg.norm(linear_vel)
        linear_stability = np.exp(-linear_speed / (velocity_threshold * 5))
        
        # 角速度稳定性（放宽）
        angular_speed = np.linalg.norm(angular_vel)
        angular_stability = np.exp(-angular_speed / (velocity_threshold * 50))
        
        # 综合稳定性
        stability = 0.5 * linear_stability + 0.5 * angular_stability
        
        return np.clip(stability, 0.0, 1.0)
    
    def compute_alignment_error(
        self,
        position_error: np.ndarray,
        orientation_error: float,
        max_position_error: float = 0.05,
        max_orientation_error: float = 0.3
    ) -> float:
        """
        计算对齐误差（转化为置信度）
        
        Args:
            position_error: 位置误差向量 (m)
            orientation_error: 姿态误差 (rad)
            max_position_error: 最大可接受位置误差
            max_orientation_error: 最大可接受姿态误差
        
        Returns:
            alignment: [0, 1] 越高对齐越好
        """
        # 位置对齐度（放宽，0.05m以内都算对齐好）
        pos_error_norm = np.linalg.norm(position_error)
        position_alignment = np.exp(-pos_error_norm / (max_position_error * 2))
        
        # 姿态对齐度（放宽）
        orientation_alignment = np.exp(-orientation_error / (max_orientation_error * 2))
        
        # 综合对齐度
        alignment = 0.5 * position_alignment + 0.5 * orientation_alignment
        
        return np.clip(alignment, 0.0, 1.0)
    
    def compute_b_readiness(
        self,
        b_distance_to_target: float,
        b_velocity: float,
        max_distance: float = 0.3
    ) -> float:
        """
        计算B臂准备度
        
        Args:
            b_distance_to_target: B臂到目标点距离 (m)
            b_velocity: B臂当前速度 (m/s)
            max_distance: 最大距离阈值
        
        Returns:
            readiness: [0, 1] 越高准备越好
        """
        # 距离准备度（放宽，0.1m以内都算好）
        distance_readiness = np.exp(-b_distance_to_target / (max_distance * 0.5))
        
        # 速度准备度（移动中=准备好了，放宽映射）
        velocity_bonus = min(b_velocity / 0.05, 0.3)  # 最多加0.3
        
        # 综合准备度
        readiness = 0.7 * distance_readiness + 0.3 + velocity_bonus
        
        return np.clip(readiness, 0.0, 1.0)
    
    def compute_hcs(
        self,
        grasp_stability: float,
        velocity_stability: float,
        alignment: float,
        b_readiness: float
    ) -> Tuple[float, HandoffStrategy]:
        """
        计算综合HCS并决定策略
        
        Args:
            grasp_stability: 抓取稳定性
            velocity_stability: 速度稳定性
            alignment: 对齐度
            b_readiness: B臂准备度
        
        Returns:
            hcs: Handoff Confidence Score
            strategy: 推荐策略
        """
        # 加权计算HCS
        hcs = (
            self.weights['grasp_stability'] * grasp_stability +
            self.weights['object_velocity'] * velocity_stability +
            self.weights['alignment_error'] * alignment +
            self.weights['b_readiness'] * b_readiness
        )
        
        # 平滑处理（避免抖动）
        self.hcs_history.append(hcs)
        if len(self.hcs_history) > self.history_size:
            self.hcs_history.pop(0)
        
        # 使用指数移动平均
        smoothed_hcs = self._exponential_moving_average(self.hcs_history)
        
        # 决定策略
        strategy = self._decide_strategy(smoothed_hcs)
        
        return smoothed_hcs, strategy
    
    def _exponential_moving_average(self, data: list, alpha: float = 0.3) -> float:
        """指数移动平均"""
        if not data:
            return 0.0
        
        ema = data[0]
        for value in data[1:]:
            ema = alpha * value + (1 - alpha) * ema
        
        return ema
    
    def _decide_strategy(self, hcs: float) -> HandoffStrategy:
        """根据HCS决定策略"""
        if hcs >= self.thresholds['fast_transfer']:
            return HandoffStrategy.FAST_TRANSFER
        elif hcs >= self.thresholds['slow_align']:
            return HandoffStrategy.SLOW_ALIGN
        elif hcs >= self.thresholds['pause_replan']:
            return HandoffStrategy.PAUSE_REPLAN
        else:
            return HandoffStrategy.EMERGENCY_STOP
    
    def update_thresholds(self, new_thresholds: Dict[str, float]):
        """动态更新阈值"""
        self.thresholds.update(new_thresholds)


class AdaptiveDecisionPolicy:
    """
    自适应决策策略
    
    根据HCS和当前状态，决定执行策略
    """
    
    def __init__(self):
        """初始化决策策略"""
        # 策略参数
        self.params = {
            HandoffStrategy.FAST_TRANSFER: {
                'speed_factor': 1.0,           # 全速
                'approach_angle': 0.0,         # 不调整
                'grip_force_factor': 1.0,      # 正常力
                'replanning': False            # 不重规划
            },
            HandoffStrategy.SLOW_ALIGN: {
                'speed_factor': 0.5,           # 半速
                'approach_angle': 0.1,         # 微调角度
                'grip_force_factor': 1.2,      # 增加力
                'replanning': False            # 不重规划
            },
            HandoffStrategy.PAUSE_REPLAN: {
                'speed_factor': 0.1,           # 极慢
                'approach_angle': 0.3,         # 大调整
                'grip_force_factor': 1.5,      # 显著增加力
                'replanning': True             # 重规划
            },
            HandoffStrategy.EMERGENCY_STOP: {
                'speed_factor': 0.0,           # 停止
                'approach_angle': 0.0,         # 不动
                'grip_force_factor': 2.0,      # 最大力
                'replanning': True             # 重规划
            }
        }
    
    def get_action_params(self, strategy: HandoffStrategy) -> Dict:
        """
        获取当前策略的执行参数
        
        Args:
            strategy: 当前策略
        
        Returns:
            params: 执行参数字典
        """
        return self.params[strategy].copy()
    
    def should_replan(self, strategy: HandoffStrategy) -> bool:
        """判断是否需要重规划"""
        return self.params[strategy]['replanning']
    
    def compute_speed_factor(self, strategy: HandoffStrategy, hcs: float) -> float:
        """
        计算动态速度因子
        
        Args:
            strategy: 当前策略
            hcs: 当前HCS值
        
        Returns:
            speed_factor: 速度因子 [0, 1]
        """
        base_speed = self.params[strategy]['speed_factor']
        
        # 根据HCS微调速度
        if strategy == HandoffStrategy.SLOW_ALIGN:
            # HCS越低，速度越慢
            speed_factor = base_speed * (hcs / 0.8)
        else:
            speed_factor = base_speed
        
        return np.clip(speed_factor, 0.0, 1.0)


class OnlineRecoveryPlanner:
    """
    在线恢复重规划器
    
    当HCS下降时，执行局部调整而不是完全重置
    """
    
    def __init__(self):
        """初始化恢复规划器"""
        # 恢复策略库
        self.recovery_strategies = {
            'reposition': self._reposition_strategy,
            'realign': self._realign_strategy,
            'adjust_force': self._adjust_force_strategy,
            'pause_stabilize': self._pause_stabilize_strategy
        }
        
        # 恢复历史
        self.recovery_history = []
        self.max_recovery_attempts = 3
    
    def plan_recovery(
        self,
        current_state: Dict,
        hcs: float,
        strategy: HandoffStrategy
    ) -> Dict:
        """
        规划恢复动作
        
        Args:
            current_state: 当前状态
            hcs: 当前HCS值
            strategy: 当前策略
        
        Returns:
            recovery_plan: 恢复计划
        """
        # 分析失败原因
        failure_reason = self._analyze_failure(current_state, hcs)
        
        # 选择恢复策略
        recovery_action = self._select_recovery_strategy(failure_reason, strategy)
        
        # 记录恢复历史
        self.recovery_history.append({
            'timestamp': current_state.get('time', 0),
            'hcs': hcs,
            'reason': failure_reason,
            'action': recovery_action
        })
        
        return {
            'failure_reason': failure_reason,
            'recovery_action': recovery_action,
            'parameters': self._compute_recovery_params(current_state, failure_reason)
        }
    
    def _analyze_failure(self, state: Dict, hcs: float) -> str:
        """分析失败原因"""
        # 检查各分量
        if state.get('grasp_stability', 1.0) < 0.5:
            return 'unstable_grasp'
        elif state.get('velocity_stability', 1.0) < 0.5:
            return 'object_moving'
        elif state.get('alignment', 1.0) < 0.5:
            return 'misalignment'
        elif state.get('b_readiness', 1.0) < 0.5:
            return 'b_not_ready'
        else:
            return 'general_low_confidence'
    
    def _select_recovery_strategy(
        self,
        failure_reason: str,
        current_strategy: HandoffStrategy
    ) -> str:
        """选择恢复策略"""
        recovery_map = {
            'unstable_grasp': 'adjust_force',
            'object_moving': 'pause_stabilize',
            'misalignment': 'realign',
            'b_not_ready': 'reposition',
            'general_low_confidence': 'pause_stabilize'
        }
        
        return recovery_map.get(failure_reason, 'pause_stabilize')
    
    def _compute_recovery_params(
        self,
        state: Dict,
        failure_reason: str
    ) -> Dict:
        """计算恢复参数"""
        if failure_reason == 'unstable_grasp':
            return {
                'force_increase': 0.5,  # 增加0.5N
                'stabilization_time': 0.5  # 稳定0.5秒
            }
        elif failure_reason == 'object_moving':
            return {
                'pause_duration': 1.0,  # 暂停1秒
                'velocity_threshold': 0.01  # 速度阈值
            }
        elif failure_reason == 'misalignment':
            return {
                'position_correction': 0.02,  # 修正2cm
                'angle_correction': 0.1  # 修正0.1rad
            }
        elif failure_reason == 'b_not_ready':
            return {
                'wait_timeout': 2.0,  # 等待2秒
                'readiness_threshold': 0.7
            }
        else:
            return {
                'pause_duration': 0.5,
                'recheck_interval': 0.1
            }
    
    def _reposition_strategy(self, params: Dict) -> str:
        """重定位策略"""
        return "A臂微调位置，等待B臂准备"
    
    def _realign_strategy(self, params: Dict) -> str:
        """重新对齐策略"""
        return "调整B臂接近角度，重新对齐"
    
    def _adjust_force_strategy(self, params: Dict) -> str:
        """调整力策略"""
        return "增加夹持力，稳定抓取"
    
    def _pause_stabilize_strategy(self, params: Dict) -> str:
        """暂停稳定策略"""
        return "暂停等待物体稳定"
    
    def get_recovery_count(self) -> int:
        """获取恢复次数"""
        return len(self.recovery_history)
    
    def reset(self):
        """重置恢复历史"""
        self.recovery_history = []


class UAHPController:
    """
    UAHP控制器
    
    整合三层架构：
    1. Belief State (HCS)
    2. Adaptive Decision Policy
    3. Online Recovery Replanning
    """
    
    def __init__(self):
        """初始化UAHP控制器"""
        self.hcs_computer = HCSComputer()
        self.decision_policy = AdaptiveDecisionPolicy()
        self.recovery_planner = OnlineRecoveryPlanner()
        
        # 状态记录
        self.belief_history = []
        self.current_belief = None
        
        # 统计
        self.stats = {
            'total_decisions': 0,
            'fast_transfers': 0,
            'slow_aligns': 0,
            'pause_replans': 0,
            'emergency_stops': 0,
            'recoveries': 0
        }
    
    def update(
        self,
        grip_force: float,
        object_mass: float,
        force_variance: float,
        linear_vel: np.ndarray,
        angular_vel: np.ndarray,
        position_error: np.ndarray,
        orientation_error: float,
        b_distance: float,
        b_velocity: float,
        current_time: float
    ) -> BeliefState:
        """
        更新信念状态
        
        Args:
            各传感器输入
        
        Returns:
            belief: 更新后的信念状态
        """
        # 计算各分量
        grasp_stability = self.hcs_computer.compute_grasp_stability(
            grip_force, object_mass, force_variance
        )
        
        velocity_stability = self.hcs_computer.compute_velocity_stability(
            linear_vel, angular_vel
        )
        
        alignment = self.hcs_computer.compute_alignment_error(
            position_error, orientation_error
        )
        
        b_readiness = self.hcs_computer.compute_b_readiness(
            b_distance, b_velocity
        )
        
        # 计算HCS和策略
        hcs, strategy = self.hcs_computer.compute_hcs(
            grasp_stability, velocity_stability, alignment, b_readiness
        )
        
        # 创建信念状态
        belief = BeliefState(
            hcs=hcs,
            grasp_stability=grasp_stability,
            object_velocity=velocity_stability,
            alignment_error=alignment,
            b_readiness=b_readiness,
            strategy=strategy,
            timestamp=current_time
        )
        
        # 更新历史
        self.belief_history.append(belief)
        self.current_belief = belief
        
        # 更新统计
        self._update_stats(strategy)
        
        return belief
    
    def get_action(self, belief: BeliefState) -> Dict:
        """
        根据信念状态获取执行动作
        
        Args:
            belief: 当前信念状态
        
        Returns:
            action: 执行动作参数
        """
        # 获取基础动作参数
        action = self.decision_policy.get_action_params(belief.strategy)
        
        # 计算动态速度因子
        action['speed_factor'] = self.decision_policy.compute_speed_factor(
            belief.strategy, belief.hcs
        )
        
        # 添加策略信息
        action['strategy'] = belief.strategy.value
        action['hcs'] = belief.hcs
        
        # 如果需要恢复，添加恢复计划
        if self.decision_policy.should_replan(belief.strategy):
            recovery_plan = self.recovery_planner.plan_recovery(
                {
                    'grasp_stability': belief.grasp_stability,
                    'velocity_stability': belief.object_velocity,
                    'alignment': belief.alignment_error,
                    'b_readiness': belief.b_readiness,
                    'time': belief.timestamp
                },
                belief.hcs,
                belief.strategy
            )
            action['recovery_plan'] = recovery_plan
            self.stats['recoveries'] += 1
        
        return action
    
    def _update_stats(self, strategy: HandoffStrategy):
        """更新统计信息"""
        self.stats['total_decisions'] += 1
        
        if strategy == HandoffStrategy.FAST_TRANSFER:
            self.stats['fast_transfers'] += 1
        elif strategy == HandoffStrategy.SLOW_ALIGN:
            self.stats['slow_aligns'] += 1
        elif strategy == HandoffStrategy.PAUSE_REPLAN:
            self.stats['pause_replans'] += 1
        elif strategy == HandoffStrategy.EMERGENCY_STOP:
            self.stats['emergency_stops'] += 1
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        stats = self.stats.copy()
        stats['avg_hcs'] = np.mean([b.hcs for b in self.belief_history]) if self.belief_history else 0
        stats['min_hcs'] = np.min([b.hcs for b in self.belief_history]) if self.belief_history else 0
        stats['recovery_rate'] = stats['recoveries'] / max(stats['total_decisions'], 1)
        return stats
    
    def reset(self):
        """重置控制器"""
        self.belief_history = []
        self.current_belief = None
        self.recovery_planner.reset()
        self.stats = {
            'total_decisions': 0,
            'fast_transfers': 0,
            'slow_aligns': 0,
            'pause_replans': 0,
            'emergency_stops': 0,
            'recoveries': 0
        }


# 测试代码
if __name__ == "__main__":
    # 创建控制器
    controller = UAHPController()
    
    # 模拟数据
    print("🧪 测试UAHP控制器...")
    print("=" * 50)
    
    # 模拟一系列状态更新
    test_cases = [
        # (grip_force, object_mass, force_variance, linear_vel, angular_vel, pos_error, orient_error, b_dist, b_vel)
        (15.0, 0.5, 0.1, np.array([0.01, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]), np.array([0.01, 0.0, 0.0]), 0.05, 0.1, 0.05),
        (12.0, 0.5, 0.3, np.array([0.05, 0.0, 0.0]), np.array([0.0, 0.0, 0.1]), np.array([0.02, 0.0, 0.0]), 0.1, 0.15, 0.03),
        (8.0, 0.5, 0.5, np.array([0.1, 0.0, 0.0]), np.array([0.0, 0.0, 0.3]), np.array([0.05, 0.0, 0.0]), 0.2, 0.2, 0.01),
        (5.0, 0.5, 1.0, np.array([0.2, 0.0, 0.0]), np.array([0.0, 0.0, 0.5]), np.array([0.1, 0.0, 0.0]), 0.5, 0.3, 0.005),
    ]
    
    for i, (gf, om, fv, lv, av, pe, oe, bd, bv) in enumerate(test_cases):
        belief = controller.update(
            grip_force=gf,
            object_mass=om,
            force_variance=fv,
            linear_vel=lv,
            angular_vel=av,
            position_error=pe,
            orientation_error=oe,
            b_distance=bd,
            b_velocity=bv,
            current_time=i * 0.1
        )
        
        action = controller.get_action(belief)
        
        print(f"\n📊 测试用例 {i+1}:")
        print(f"  HCS: {belief.hcs:.3f}")
        print(f"  策略: {belief.strategy.value}")
        print(f"  速度因子: {action['speed_factor']:.3f}")
        if 'recovery_plan' in action:
            print(f"  恢复计划: {action['recovery_plan']['recovery_action']}")
    
    # 打印统计
    print("\n" + "=" * 50)
    print("📈 统计信息:")
    stats = controller.get_stats()
    for key, value in stats.items():
        if isinstance(value, float):
            print(f"  {key}: {value:.3f}")
        else:
            print(f"  {key}: {value}")
    
    print("\n✅ UAHP控制器测试完成！")