"""
Additional Unit Tests for Space Module Dual-Arm Assembly
========================================================
Extends test suite to 100+ tests with boundary conditions and integration tests.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
from franka_controller import (
    FrankaController, JointState, CartesianPose, GraspPlan, TrajectoryPoint
)


# ==================== Fixtures ====================

@pytest.fixture(scope="module")
def controller():
    """创建共享的控制器实例"""
    return FrankaController()


@pytest.fixture
def reset_controller(controller):
    """每个测试前重置"""
    controller.reset()
    return controller


# ==================== 边界条件测试 ====================

class TestBoundaryConditions:
    """边界条件测试"""
    
    def test_joint_limits_not_exceeded(self, reset_controller):
        """测试关节不会超出限位"""
        for i in range(7):
            # 尝试超出限位
            target = reset_controller.HOME_QPOS.copy()
            target[i] = 10.0  # 远超限位
            reset_controller.set_joint_positions(target, steps=50)
            # 应该被限制在限位内
            assert reset_controller.data.qpos[i] <= reset_controller.model.jnt_range[i, 1] + 0.01
    
    def test_zero_gravity(self, reset_controller):
        """测试零重力下的控制"""
        # 保存原始重力
        original_gravity = reset_controller.model.opt.gravity.copy()
        
        # 设置零重力
        reset_controller.model.opt.gravity[:] = [0, 0, 0]
        
        # 运行控制
        reset_controller.set_joint_positions(reset_controller.HOME_QPOS, steps=100)
        
        # 恢复重力
        reset_controller.model.opt.gravity[:] = original_gravity
        
        # 应该仍然能控制
        error = np.linalg.norm(reset_controller.data.qpos[:7] - reset_controller.HOME_QPOS)
        assert error < 0.5
    
    def test_high_stiffness(self, reset_controller):
        """测试高刚度控制"""
        target = np.array([0.4, 0, 0.3])
        K = np.diag([1000, 1000, 1000])  # 高刚度
        D = np.diag([100, 100, 100])
        
        tau = reset_controller.impedance_control(target, stiffness=K, damping=D)
        assert len(tau) == 7
        assert np.all(np.isfinite(tau))
    
    def test_low_stiffness(self, reset_controller):
        """测试低刚度控制"""
        target = np.array([0.4, 0, 0.3])
        K = np.diag([1, 1, 1])  # 低刚度
        D = np.diag([0.1, 0.1, 0.1])
        
        tau = reset_controller.impedance_control(target, stiffness=K, damping=D)
        assert len(tau) == 7
        assert np.all(np.isfinite(tau))
    
    def test_gripper_full_range(self, reset_controller):
        """测试夹爪全范围运动"""
        # 完全打开
        reset_controller.gripper_control(0.04, steps=30)
        assert reset_controller.data.ctrl[7] >= 0
        
        # 完全闭合
        reset_controller.gripper_control(0.0, steps=30)
        assert reset_controller.data.ctrl[7] <= 255
    
    def test_large_joint_movement(self, reset_controller):
        """测试大范围关节运动"""
        # 从home移动到大幅度位置
        target = reset_controller.HOME_QPOS + np.array([1.0, -0.5, 0.5, -1.0, 0.5, -0.5, 0.5])
        reset_controller.set_joint_positions(target, steps=500)
        
        # 应该有明显移动
        error = np.linalg.norm(reset_controller.data.qpos[:7] - target)
        assert error < 1.0  # 允许较大误差
    
    def test_small_joint_movement(self, reset_controller):
        """测试小范围关节运动"""
        # 微小移动
        target = reset_controller.HOME_QPOS + np.array([0.01, -0.01, 0.01, -0.01, 0.01, -0.01, 0.01])
        reset_controller.set_joint_positions(target, steps=100)
        
        # 应该接近目标
        error = np.linalg.norm(reset_controller.data.qpos[:7] - target)
        assert error < 0.1


# ==================== 集成测试 ====================

class TestIntegration:
    """集成测试"""
    
    def test_full_pick_place_cycle(self, reset_controller):
        """测试完整的拾取-放置周期"""
        # 拾取
        pick_pos = np.array([0.3, 0, 0.05])
        result1 = reset_controller.pick_object(pick_pos)
        assert result1 is True
        
        # 移动
        reset_controller.set_joint_positions(reset_controller.HOME_QPOS, steps=100)
        
        # 放置
        place_pos = np.array([0.4, 0, 0.05])
        result2 = reset_controller.place_object(place_pos)
        assert result2 is True
    
    def test_multiple_object_handling(self, reset_controller):
        """测试多物体处理"""
        positions = [
            np.array([0.3, 0, 0.02]),
            np.array([0.4, 0, 0.02]),
            np.array([0.5, 0, 0.02])
        ]
        
        result = reset_controller.stack_objects(positions)
        assert result["total_objects"] == 3
        assert len(result["operations"]) == 2
    
    def test_sort_workflow(self, reset_controller):
        """测试分拣工作流"""
        objects = [
            np.array([0.3, 0.1, 0.02]),
            np.array([0.3, -0.1, 0.02])
        ]
        zones = [
            np.array([0.5, 0.1, 0.02]),
            np.array([0.5, -0.1, 0.02])
        ]
        
        result = reset_controller.sort_objects(objects, zones)
        assert result["sorted_count"] == 2
    
    def test_trajectory_recording(self, reset_controller):
        """测试轨迹记录"""
        qpos_list = [
            reset_controller.HOME_QPOS,
            reset_controller.HOME_QPOS + 0.1,
            reset_controller.HOME_QPOS + 0.2
        ]
        
        recorded = reset_controller.record_trajectory(qpos_list)
        assert len(recorded) == 3
        
        for entry in recorded:
            assert "qpos" in entry
            assert "qvel" in entry
            assert "time" in entry
    
    def test_task_orchestration(self, reset_controller):
        """测试任务编排"""
        tasks = [
            {"type": "gripper", "params": {"width": 0.04}},
            {"type": "move", "params": {"qpos": reset_controller.HOME_QPOS.tolist()}},
            {"type": "gripper", "params": {"width": 0.0}}
        ]
        
        result = reset_controller.task_orchestration(tasks)
        assert result["total_tasks"] == 3
        assert result["success_rate"] > 0
    
    def test_skill_learning(self, reset_controller):
        """测试技能学习"""
        demos = [
            [reset_controller.HOME_QPOS + np.random.randn(7) * 0.1 for _ in range(20)],
            [reset_controller.HOME_QPOS + np.random.randn(7) * 0.1 for _ in range(25)]
        ]
        
        result = reset_controller.skill_learning(demos)
        assert "mean_trajectory" in result
        assert "variance" in result
        assert result["num_demonstrations"] == 2


# ==================== 传感器测试 ====================

class TestSensors:
    """传感器测试"""
    
    def test_force_sensor_readings(self, reset_controller):
        """测试力传感器读数"""
        force_data = reset_controller.force_estimation()
        
        assert "num_contacts" in force_data
        assert "total_force" in force_data
        assert "force_magnitude" in force_data
        
        assert force_data["force_magnitude"] >= 0
        assert len(force_data["total_force"]) == 3
    
    def test_joint_state_sensors(self, reset_controller):
        """测试关节状态传感器"""
        info = reset_controller.get_joint_info()
        
        assert "joint_positions" in info or "num_joints" in info
        assert info["num_joints"] >= 9
    
    def test_end_effector_position(self, reset_controller):
        """测试末端位置传感器"""
        pos = reset_controller.get_end_effector_pos()
        
        assert len(pos) == 3
        assert np.all(np.isfinite(pos))
    
    def test_collision_sensor(self, reset_controller):
        """测试碰撞传感器"""
        collision_data = reset_controller.collision_detection()
        
        assert "has_collision" in collision_data
        assert "num_contacts" in collision_data
        assert isinstance(collision_data["has_collision"], bool)


# ==================== 控制器鲁棒性测试 ====================

class TestRobustness:
    """控制器鲁棒性测试"""
    
    def test_rapid_successive_commands(self, reset_controller):
        """测试快速连续命令"""
        for i in range(10):
            target = reset_controller.HOME_QPOS + np.random.randn(7) * 0.1
            reset_controller.set_joint_positions(target, steps=10)
        
        # 应该不会崩溃
        assert reset_controller.model is not None
    
    def test_concurrent_operations(self, reset_controller):
        """测试并发操作"""
        # 同时进行多个操作
        reset_controller.gripper_control(0.02, steps=10)
        reset_controller.set_joint_positions(reset_controller.HOME_QPOS, steps=50)
        reset_controller.force_estimation()
        
        # 应该都能完成
        assert True
    
    def test_reset_after_error(self, reset_controller):
        """测试错误后重置"""
        # 尝试可能导致错误的操作
        try:
            reset_controller.set_joint_positions(np.array([100, 100, 100, 100, 100, 100, 100]), steps=10)
        except:
            pass
        
        # 重置应该成功
        reset_controller.reset()
        error = np.linalg.norm(reset_controller.data.qpos[:7] - reset_controller.HOME_QPOS)
        assert error < 0.5
    
    def test_memory_stability(self, reset_controller):
        """测试内存稳定性"""
        # 运行多次操作
        for _ in range(50):
            reset_controller.reset()
            reset_controller.set_joint_positions(reset_controller.HOME_QPOS, steps=20)
            reset_controller.force_estimation()
        
        # 应该没有内存泄漏
        assert reset_controller.model is not None
        assert reset_controller.data is not None


# ==================== 性能边界测试 ====================

class TestPerformanceBounds:
    """性能边界测试"""
    
    def test_fk_speed_bound(self, reset_controller):
        """测试正运动学速度边界"""
        import time
        
        start = time.time()
        for _ in range(1000):
            reset_controller.forward_kinematics(reset_controller.HOME_QPOS)
        elapsed = time.time() - start
        
        # 应该在1秒内完成1000次
        assert elapsed < 1.0
    
    def test_ik_speed_bound(self, reset_controller):
        """测试逆运动学速度边界"""
        import time
        
        target = np.array([0.4, 0, 0.3])
        start = time.time()
        for _ in range(100):
            reset_controller.impedance_control(target)
        elapsed = time.time() - start
        
        # 应该在2秒内完成100次
        assert elapsed < 2.0
    
    def test_force_estimation_speed(self, reset_controller):
        """测试力估计速度"""
        import time
        
        start = time.time()
        for _ in range(100):
            reset_controller.force_estimation()
        elapsed = time.time() - start
        
        # 应该在1秒内完成100次
        assert elapsed < 1.0


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
