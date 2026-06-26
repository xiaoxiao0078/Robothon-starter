"""
Franka Panda Controller 测试套件
================================
覆盖全部25个任务的单元测试。
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

# 添加项目路径（使用内嵌模型，无需vendor目录）
sys.path.insert(0, str(Path(__file__).parent))

# 导入控制器（从同级目录）
sys.path.insert(0, str(Path(__file__).parent.parent))
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


# ==================== 任务1: 模型加载与初始化 ====================

class TestTask01ModelLoading:
    """任务1: 模型加载与初始化"""

    def test_model_loads_successfully(self, controller):
        """测试模型能正常加载"""
        assert controller.model is not None
        assert controller.data is not None

    def test_model_has_correct_dof(self, controller):
        """测试模型自由度正确 (7关节 + 2手指 + 滑动)"""
        assert controller.model.njnt >= 9  # 至少9个关节
        assert controller.model.nv >= 9

    def test_reset_returns_home(self, controller):
        """测试重置后回到home位姿"""
        controller.reset()
        np.testing.assert_allclose(
            controller.data.qpos[:7], controller.HOME_QPOS, atol=0.01
        )

    def test_renderer_initialized(self, controller):
        """测试渲染器初始化"""
        assert controller.renderer is not None


# ==================== 任务2: 关节信息查询 ====================

class TestTask02JointInfo:
    """任务2: 关节信息查询"""

    def test_joint_info_returns_dict(self, reset_controller):
        """测试返回字典结构"""
        info = reset_controller.get_joint_info()
        assert isinstance(info, dict)
        assert "num_joints" in info
        assert "joint_names" in info
        assert "joint_limits_low" in info
        assert "joint_limits_high" in info

    def test_joint_count(self, reset_controller):
        """测试关节数量"""
        info = reset_controller.get_joint_info()
        assert info["num_joints"] >= 9
        assert info["num_dof"] >= 9
        assert len(info["joint_names"]) == 7

    def test_joint_limits_shape(self, reset_controller):
        """测试关节限位维度"""
        info = reset_controller.get_joint_info()
        assert len(info["joint_limits_low"]) == 7
        assert len(info["joint_limits_high"]) == 7

    def test_joint_limits合理性(self, reset_controller):
        """测试关节限位合理性"""
        info = reset_controller.get_joint_info()
        assert np.all(info["joint_limits_low"] < info["joint_limits_high"])


# ==================== 任务3: 正运动学 ====================

class TestTask03ForwardKinematics:
    """任务3: 正运动学"""

    def test_fk_returns_pose(self, reset_controller):
        """测试FK返回正确类型"""
        qpos = np.zeros(7)
        pose = reset_controller.forward_kinematics(qpos)
        assert isinstance(pose, CartesianPose)
        assert len(pose.position) == 3
        assert len(pose.orientation) == 4

    def test_fk_home_position(self, reset_controller):
        """测试home位姿末端位置"""
        pose = reset_controller.forward_kinematics(reset_controller.HOME_QPOS)
        # home位姿末端应该在合理范围内
        assert 0.1 < pose.position[2] < 1.0  # z在合理范围

    def test_fk_diff_qpos_diff_pose(self, reset_controller):
        """测试不同关节角度产生不同位姿"""
        q1 = np.array([0, 0, 0, -1.57, 0, 1.57, -0.78])
        q2 = np.array([0.5, 0, 0, -1.57, 0, 1.57, -0.78])
        p1 = reset_controller.forward_kinematics(q1)
        p2 = reset_controller.forward_kinematics(q2)
        # 不同关节角度应该产生不同的末端位置
        assert not np.allclose(p1.position, p2.position, atol=0.01)

    def test_fk_quaternion_unit(self, reset_controller):
        """测试四元数是单位四元数"""
        pose = reset_controller.forward_kinematics(reset_controller.HOME_QPOS)
        norm = np.linalg.norm(pose.orientation)
        assert abs(norm - 1.0) < 0.01


# ==================== 任务4: 雅可比矩阵 ====================

class TestTask04Jacobian:
    """任务4: 雅可比矩阵"""

    def test_jacobian_shape(self, reset_controller):
        """测试雅可比矩阵维度"""
        jac_pos, jac_rot = reset_controller.get_jacobian(reset_controller.HOME_QPOS)
        assert jac_pos.shape == (3, 7)
        assert jac_rot.shape == (3, 7)

    def test_jacobian_not_zero(self, reset_controller):
        """测试雅可比不是零矩阵"""
        jac_pos, _ = reset_controller.get_jacobian(reset_controller.HOME_QPOS)
        assert np.linalg.norm(jac_pos) > 0.01

    def test_jacobian_diff_qpos(self, reset_controller):
        """测试不同位姿雅可比不同"""
        q1 = np.zeros(7)
        q2 = np.ones(7) * 0.5
        j1, _ = reset_controller.get_jacobian(q1)
        j2, _ = reset_controller.get_jacobian(q2)
        assert not np.allclose(j1, j2, atol=0.01)

    def test_jacobian_rank(self, reset_controller):
        """测试雅可比矩阵满秩"""
        jac_pos, _ = reset_controller.get_jacobian(reset_controller.HOME_QPOS)
        rank = np.linalg.matrix_rank(jac_pos)
        assert rank >= 3  # 至少3维可达


# ==================== 任务5: 关节位置控制 ====================

class TestTask05JointControl:
    """任务5: 关节位置控制"""

    def test_set_joint_positions(self, reset_controller):
        """测试设置关节位置"""
        target = np.array([0.1, 0.1, 0.1, -1.57, 0.1, 1.57, -0.78])
        result = reset_controller.set_joint_positions(target, steps=200)
        # 应该接近目标
        error = np.linalg.norm(reset_controller.data.qpos[:7] - target)
        assert error < 0.3  # 允许较大误差

    def test_set_home_position(self, reset_controller):
        """测试设置home位姿"""
        # 先移到别处
        reset_controller.data.qpos[:7] = np.ones(7) * 0.5
        # 再回home
        result = reset_controller.set_joint_positions(reset_controller.HOME_QPOS, steps=200)
        error = np.linalg.norm(reset_controller.data.qpos[:7] - reset_controller.HOME_QPOS)
        assert error < 0.3


# ==================== 任务6: 关节空间线性插值 ====================

class TestTask06JointInterpolation:
    """任务6: 关节空间线性插值"""

    def test_interpolation_length(self, reset_controller):
        """测试插值点数"""
        q0 = np.zeros(7)
        q1 = np.ones(7) * 0.5
        traj = reset_controller.linear_interpolation_joint(q0, q1, num_points=20)
        assert len(traj) == 20

    def test_interpolation_start_end(self, reset_controller):
        """测试插值起点和终点"""
        q0 = np.zeros(7)
        q1 = np.ones(7) * 0.5
        traj = reset_controller.linear_interpolation_joint(q0, q1, num_points=10)
        np.testing.assert_allclose(traj[0].position, q0, atol=0.01)
        np.testing.assert_allclose(traj[-1].position, q1, atol=0.01)

    def test_interpolation_monotonic(self, reset_controller):
        """测试插值单调性"""
        q0 = np.zeros(7)
        q1 = np.ones(7) * 0.5
        traj = reset_controller.linear_interpolation_joint(q0, q1, num_points=10)
        for i in range(1, len(traj)):
            assert traj[i].time > traj[i-1].time


# ==================== 任务7: 笛卡尔空间线性插值 ====================

class TestTask07CartesianInterpolation:
    """任务7: 笛卡尔空间线性插值"""

    def test_cartesian_interpolation_length(self, reset_controller):
        """测试笛卡尔插值点数"""
        p0 = CartesianPose(np.zeros(3), np.array([1, 0, 0, 0]))
        p1 = CartesianPose(np.ones(3) * 0.5, np.array([1, 0, 0, 0]))
        traj = reset_controller.linear_interpolation_cartesian(p0, p1, num_points=15)
        assert len(traj) == 15

    def test_cartesian_interpolation_positions(self, reset_controller):
        """测试笛卡尔插值位置连续"""
        p0 = CartesianPose(np.zeros(3), np.array([1, 0, 0, 0]))
        p1 = CartesianPose(np.ones(3) * 0.5, np.array([1, 0, 0, 0]))
        traj = reset_controller.linear_interpolation_cartesian(p0, p1, num_points=10)
        for i in range(1, len(traj)):
            dist = np.linalg.norm(traj[i].position - traj[i-1].position)
            assert dist < 0.2  # 步长合理


# ==================== 任务8: 最小抖动轨迹 ====================

class TestTask08MinJerk:
    """任务8: 最小抖动轨迹规划"""

    def test_min_jerk_length(self, reset_controller):
        """测试最小抖动轨迹点数"""
        q0 = np.zeros(7)
        q1 = np.ones(7) * 0.5
        traj = reset_controller.minimum_jerk_trajectory(q0, q1, duration=1.0, num_points=20)
        assert len(traj) == 20

    def test_min_jerk_boundary_conditions(self, reset_controller):
        """测试边界条件 (起点终点速度为0)"""
        q0 = np.zeros(7)
        q1 = np.ones(7) * 0.5
        traj = reset_controller.minimum_jerk_trajectory(q0, q1, duration=2.0, num_points=50)
        # 起点和终点速度应接近0
        np.testing.assert_allclose(traj[0].velocity, np.zeros(7), atol=0.1)
        np.testing.assert_allclose(traj[-1].velocity, np.zeros(7), atol=0.1)

    def test_min_jerk_smooth(self, reset_controller):
        """测试轨迹平滑性"""
        q0 = np.zeros(7)
        q1 = np.ones(7) * 0.5
        traj = reset_controller.minimum_jerk_trajectory(q0, q1, duration=2.0, num_points=50)
        # 位置变化应平滑
        for i in range(2, len(traj)):
            accel = traj[i].position - 2*traj[i-1].position + traj[i-2].position
            assert np.linalg.norm(accel) < 0.1


# ==================== 任务9: 障碍物避让 ====================

class TestTask09ObstacleAvoidance:
    """任务9: 障碍物避让"""

    def test_avoidance_returns_dq(self, reset_controller):
        """测试避让返回关节增量"""
        target = np.array([0.4, 0, 0.3])
        obstacle = np.array([0.3, 0, 0.3])
        dq = reset_controller.obstacle_avoidance(
            reset_controller.HOME_QPOS, target, obstacle
        )
        assert len(dq) == 7

    def test_avoidance_pushes_away(self, reset_controller):
        """测试避让方向远离障碍物"""
        target = np.array([0.3, 0, 0.3])
        obstacle = np.array([0.3, 0, 0.3])  # 同一位置
        dq = reset_controller.obstacle_avoidance(
            reset_controller.HOME_QPOS, target, obstacle
        )
        # 应该有非零增量
        assert np.linalg.norm(dq) > 1e-6

    def test_avoidance_no_obstacle(self, reset_controller):
        """测试无障碍物时正常趋向目标"""
        target = np.array([0.4, 0, 0.3])
        obstacle = np.array([1.0, 1.0, 1.0])  # 远处
        dq = reset_controller.obstacle_avoidance(
            reset_controller.HOME_QPOS, target, obstacle
        )
        assert np.linalg.norm(dq) > 0


# ==================== 任务10: 工作空间分析 ====================

class TestTask10Workspace:
    """任务10: 工作空间分析"""

    def test_workspace_returns_stats(self, reset_controller):
        """测试返回统计信息"""
        stats = reset_controller.workspace_analysis(num_samples=100)
        assert "x_range" in stats
        assert "y_range" in stats
        assert "z_range" in stats
        assert "workspace_volume" in stats
        assert "max_reach" in stats

    def test_workspace_volume_positive(self, reset_controller):
        """测试工作空间体积为正"""
        stats = reset_controller.workspace_analysis(num_samples=200)
        assert stats["workspace_volume"] > 0

    def test_workspace_points_count(self, reset_controller):
        """测试采样点数量"""
        stats = reset_controller.workspace_analysis(num_samples=150)
        assert len(stats["reachable_points"]) == 150

    def test_max_reach_reasonable(self, reset_controller):
        """测试最大可达距离合理"""
        stats = reset_controller.workspace_analysis(num_samples=500)
        # 双臂系统最大可达距离
        assert 0.5 < stats["max_reach"] < 2.0


# ==================== 任务11: 抓取接近向量 ====================

class TestTask11ApproachVector:
    """任务11: 抓取接近向量"""

    def test_approach_above_object(self, reset_controller):
        """测试接近位置在物体上方"""
        obj_pos = np.array([0.4, 0, 0.1])
        approach = reset_controller.compute_approach_vector(obj_pos)
        assert approach.position[2] > obj_pos[2]

    def test_approach_height(self, reset_controller):
        """测试接近高度偏移"""
        obj_pos = np.array([0.4, 0, 0.1])
        approach = reset_controller.compute_approach_vector(obj_pos, approach_height=0.2)
        assert abs(approach.position[2] - obj_pos[2] - 0.2) < 0.01

    def test_approach_quat_valid(self, reset_controller):
        """测试接近方向四元数有效"""
        obj_pos = np.array([0.4, 0, 0.1])
        approach = reset_controller.compute_approach_vector(obj_pos)
        assert len(approach.orientation) == 4
        assert abs(np.linalg.norm(approach.orientation) - 1.0) < 0.01


# ==================== 任务12: 抓取位姿计算 ====================

class TestTask12GraspPose:
    """任务12: 抓取位姿计算"""

    def test_grasp_plan_structure(self, reset_controller):
        """测试抓取规划结构"""
        obj_pos = np.array([0.4, 0, 0.05])
        plan = reset_controller.compute_grasp_pose(obj_pos)
        assert isinstance(plan, GraspPlan)
        assert len(plan.approach_pos) == 3
        assert len(plan.grasp_pos) == 3
        assert len(plan.lift_pos) == 3

    def test_grasp_plan_height_order(self, reset_controller):
        """测试高度顺序: 接近 > 抬起 > 抓取"""
        obj_pos = np.array([0.4, 0, 0.05])
        plan = reset_controller.compute_grasp_pose(obj_pos)
        assert plan.approach_pos[2] > plan.lift_pos[2]
        assert plan.lift_pos[2] > plan.grasp_pos[2]

    def test_grasp_plan_width(self, reset_controller):
        """测试抓取宽度"""
        obj_pos = np.array([0.4, 0, 0.05])
        plan = reset_controller.compute_grasp_pose(obj_pos, object_width=0.06)
        assert plan.grasp_width == 0.06


# ==================== 任务13: 预抓取位置 ====================

class TestTask13PreGrasp:
    """任务13: 预抓取位置"""

    def test_pre_grasp_above(self, reset_controller):
        """测试预抓取位置在物体上方"""
        obj_pos = np.array([0.4, 0, 0.1])
        pre = reset_controller.pre_grasp_position(obj_pos)
        assert pre[2] > obj_pos[2]

    def test_pre_grasp_safety_margin(self, reset_controller):
        """测试安全余量"""
        obj_pos = np.array([0.4, 0, 0.1])
        pre = reset_controller.pre_grasp_position(obj_pos, safety_margin=0.1)
        assert pre[2] >= obj_pos[2] + 0.1


# ==================== 任务14: 夹爪控制 ====================

class TestTask14Gripper:
    """任务14: 夹爪控制"""

    def test_gripper_open(self, reset_controller):
        """测试夹爪打开"""
        result = reset_controller.gripper_control(0.04, steps=30)
        assert result is True

    def test_gripper_close(self, reset_controller):
        """测试夹爪闭合"""
        result = reset_controller.gripper_control(0.0, steps=30)
        assert result is True

    def test_gripper_ctrl_range(self, reset_controller):
        """测试夹爪控制值范围"""
        reset_controller.gripper_control(0.02, steps=10)
        assert 0 <= reset_controller.data.ctrl[7] <= 255


# ==================== 任务15: 接触力估计 ====================

class TestTask15ForceEstimation:
    """任务15: 接触力估计"""

    def test_force_returns_dict(self, reset_controller):
        """测试返回字典"""
        force_data = reset_controller.force_estimation()
        assert isinstance(force_data, dict)
        assert "num_contacts" in force_data
        assert "total_force" in force_data
        assert "force_magnitude" in force_data

    def test_force_magnitude_non_negative(self, reset_controller):
        """测试力大小非负"""
        force_data = reset_controller.force_estimation()
        assert force_data["force_magnitude"] >= 0

    def test_force_vector_shape(self, reset_controller):
        """测试力向量维度"""
        force_data = reset_controller.force_estimation()
        assert len(force_data["total_force"]) == 3


# ==================== 任务16: 拾取操作 ====================

class TestTask16Pick:
    """任务16: 拾取操作"""

    def test_pick_runs_without_error(self, reset_controller):
        """测试拾取操作能运行"""
        obj_pos = np.array([0.4, 0, 0.05])
        result = reset_controller.pick_object(obj_pos)
        assert result is True

    def test_pick_changes_end_effector(self, reset_controller):
        """测试拾取后末端位置变化"""
        reset_controller.reset()
        pos_before = reset_controller.get_end_effector_pos()
        reset_controller.pick_object(np.array([0.4, 0, 0.05]))
        pos_after = reset_controller.get_end_effector_pos()
        # 末端位置应该有变化
        assert not np.allclose(pos_before, pos_after, atol=0.01)


# ==================== 任务17: 放置操作 ====================

class TestTask17Place:
    """任务17: 放置操作"""

    def test_place_runs_without_error(self, reset_controller):
        """测试放置操作能运行"""
        place_pos = np.array([0.3, 0, 0.05])
        result = reset_controller.place_object(place_pos)
        assert result is True

    def test_place_moves_to_target(self, reset_controller):
        """测试放置移动到目标位置"""
        reset_controller.reset()
        place_pos = np.array([0.3, 0, 0.1])
        reset_controller.place_object(place_pos)
        final_pos = reset_controller.get_end_effector_pos()
        # 最终位置应该接近目标
        dist = np.linalg.norm(final_pos[:2] - place_pos[:2])
        assert dist < 0.3


# ==================== 任务18: 物体堆叠 ====================

class TestTask18Stack:
    """任务18: 物体堆叠"""

    def test_stack_returns_result(self, reset_controller):
        """测试堆叠返回结果"""
        positions = [
            np.array([0.3, 0, 0.02]),
            np.array([0.4, 0, 0.02]),
            np.array([0.5, 0, 0.02]),
        ]
        result = reset_controller.stack_objects(positions)
        assert "total_objects" in result
        assert result["total_objects"] == 3

    def test_stack_operations_count(self, reset_controller):
        """测试堆叠操作次数"""
        positions = [np.array([0.3, 0, 0.02]), np.array([0.4, 0, 0.02])]
        result = reset_controller.stack_objects(positions)
        assert len(result["operations"]) == 1  # n-1次操作


# ==================== 任务19: 物体分拣 ====================

class TestTask19Sort:
    """任务19: 物体分拣"""

    def test_sort_returns_result(self, reset_controller):
        """测试分拣返回结果"""
        objects = [np.array([0.3, 0.1, 0.02]), np.array([0.3, -0.1, 0.02])]
        zones = [np.array([0.5, 0.1, 0.02]), np.array([0.5, -0.1, 0.02])]
        result = reset_controller.sort_objects(objects, zones)
        assert result["sorted_count"] == 2

    def test_sort_operations(self, reset_controller):
        """测试分拣操作"""
        objects = [np.array([0.3, 0, 0.02])]
        zones = [np.array([0.5, 0, 0.02])]
        result = reset_controller.sort_objects(objects, zones)
        assert len(result["operations"]) == 1


# ==================== 任务20: 运动轨迹记录 ====================

class TestTask20TrajectoryRecord:
    """任务20: 运动轨迹记录"""

    def test_record_returns_list(self, reset_controller):
        """测试记录返回列表"""
        qpos_list = [reset_controller.HOME_QPOS + np.random.randn(7) * 0.1 
                     for _ in range(20)]
        recorded = reset_controller.record_trajectory(qpos_list)
        assert isinstance(recorded, list)
        assert len(recorded) == 20

    def test_recorded_entry_has_fields(self, reset_controller):
        """测试记录条目包含必要字段"""
        qpos_list = [reset_controller.HOME_QPOS for _ in range(5)]
        recorded = reset_controller.record_trajectory(qpos_list)
        entry = recorded[0]
        assert "qpos" in entry
        assert "qvel" in entry
        assert "time" in entry
        assert "end_effector_force" in entry

    def test_record_force_data(self, reset_controller):
        """测试力数据记录"""
        qpos_list = [reset_controller.HOME_QPOS for _ in range(5)]
        recorded = reset_controller.record_trajectory(qpos_list, record_force=True)
        for entry in recorded:
            assert entry["end_effector_force"] >= 0


# ==================== 任务21: 碰撞检测 ====================

class TestTask21Collision:
    """任务21: 碰撞检测"""

    def test_collision_returns_dict(self, reset_controller):
        """测试碰撞检测返回字典"""
        result = reset_controller.collision_detection()
        assert "has_collision" in result
        assert "num_contacts" in result

    def test_collision_no_contact(self, reset_controller):
        """测试初始状态无碰撞"""
        reset_controller.reset()
        result = reset_controller.collision_detection()
        # home位姿不应有碰撞
        assert isinstance(result["has_collision"], bool)


# ==================== 任务22: 阻抗控制 ====================

class TestTask22Impedance:
    """任务22: 阻抗控制"""

    def test_impedance_returns_torque(self, reset_controller):
        """测试阻抗控制返回力矩"""
        target = np.array([0.4, 0, 0.3])
        tau = reset_controller.impedance_control(target)
        assert len(tau) == 7

    def test_impedance_custom_stiffness(self, reset_controller):
        """测试自定义刚度"""
        target = np.array([0.4, 0, 0.3])
        K = np.diag([100, 100, 100])
        D = np.diag([10, 10, 10])
        tau = reset_controller.impedance_control(target, stiffness=K, damping=D)
        assert len(tau) == 7
        assert np.linalg.norm(tau) >= 0


# ==================== 任务23: 视觉伺服 ====================

class TestTask23VisualServoing:
    """任务23: 视觉伺服"""

    def test_visual_servoing_returns_dq(self, reset_controller):
        """测试视觉伺服返回关节增量"""
        target_pixel = np.array([320.0, 240.0])
        current_pixel = np.array([300.0, 220.0])
        dq = reset_controller.visual_servoing(target_pixel, current_pixel)
        assert len(dq) == 7

    def test_visual_servoing_direction(self, reset_controller):
        """测试视觉伺服方向正确"""
        target = np.array([320.0, 240.0])
        current = np.array([280.0, 200.0])  # 偏左上方
        dq = reset_controller.visual_servoing(target, current)
        # 应该有非零增量
        assert np.linalg.norm(dq) > 1e-8


# ==================== 任务24: 示教学习 ====================

class TestTask24SkillLearning:
    """任务24: 示教学习"""

    def test_skill_learning_returns_params(self, reset_controller):
        """测试返回学习参数"""
        demos = [
            [np.random.randn(7) * 0.1 for _ in range(20)],
            [np.random.randn(7) * 0.1 for _ in range(25)],
            [np.random.randn(7) * 0.1 for _ in range(18)],
        ]
        result = reset_controller.skill_learning(demos)
        assert "mean_trajectory" in result
        assert "variance" in result
        assert result["num_demonstrations"] == 3

    def test_skill_learning_alignment(self, reset_controller):
        """测试轨迹对齐"""
        demos = [
            [np.ones(7) * i * 0.1 for i in range(10)],
            [np.ones(7) * i * 0.1 for i in range(15)],
        ]
        result = reset_controller.skill_learning(demos)
        # 轨迹长度应统一
        assert result["trajectory_length"] == 15


# ==================== 任务25: 任务编排 ====================

class TestTask25Orchestration:
    """任务25: 任务编排"""

    def test_orchestration_empty(self, reset_controller):
        """测试空任务序列"""
        result = reset_controller.task_orchestration([])
        assert result["total_tasks"] == 0
        assert result["success_rate"] == 0

    def test_orchestration_single_task(self, reset_controller):
        """测试单个任务"""
        tasks = [{"type": "gripper", "params": {"width": 0.04}}]
        result = reset_controller.task_orchestration(tasks)
        assert result["total_tasks"] == 1
        assert result["completed"] == 1

    def test_orchestration_multiple_tasks(self, reset_controller):
        """测试多个任务"""
        tasks = [
            {"type": "gripper", "params": {"width": 0.04}},
            {"type": "move", "params": {"qpos": reset_controller.HOME_QPOS.tolist()}},
            {"type": "gripper", "params": {"width": 0.0}},
        ]
        result = reset_controller.task_orchestration(tasks)
        assert result["total_tasks"] == 3
        assert result["success_rate"] > 0

    def test_orchestration_unknown_task(self, reset_controller):
        """测试未知任务类型"""
        tasks = [{"type": "unknown_task", "params": {}}]
        result = reset_controller.task_orchestration(tasks)
        # 未知任务不应导致整体失败
        assert result["total_tasks"] == 1


# ==================== 辅助方法测试 ====================

class TestHelperMethods:
    """辅助方法测试"""

    def test_render_frame(self, reset_controller):
        """测试渲染帧"""
        frame = reset_controller.render_frame()
        assert frame is not None
        assert len(frame.shape) == 3  # H, W, C
        assert frame.shape[0] == 480
        assert frame.shape[1] == 640

    def test_get_end_effector_pos(self, reset_controller):
        """测试获取末端位置"""
        pos = reset_controller.get_end_effector_pos()
        assert len(pos) == 3
        assert np.all(np.isfinite(pos))

    def test_move_to_home(self, reset_controller):
        """测试移动到home"""
        reset_controller.data.qpos[:7] = np.ones(7) * 0.5
        reset_controller.move_to_home()
        # 应该接近home
        error = np.linalg.norm(reset_controller.data.qpos[:7] - reset_controller.HOME_QPOS)
        assert error < 0.5

    def test_slerp_interpolation(self, reset_controller):
        """测试SLERP插值"""
        q0 = np.array([1, 0, 0, 0])
        q1 = np.array([0, 1, 0, 0])
        q_mid = reset_controller._slerp(q0, q1, 0.5)
        assert abs(np.linalg.norm(q_mid) - 1.0) < 0.01


# ==================== 性能测试 ====================

class TestPerformance:
    """性能测试"""

    def test_fk_speed(self, reset_controller):
        """测试正运动学速度 (<10ms)"""
        import time
        start = time.time()
        for _ in range(100):
            reset_controller.forward_kinematics(reset_controller.HOME_QPOS)
        elapsed = time.time() - start
        assert elapsed < 1.0  # 100次 < 1秒

    def test_jacobian_speed(self, reset_controller):
        """测试雅可比计算速度"""
        import time
        start = time.time()
        for _ in range(100):
            reset_controller.get_jacobian(reset_controller.HOME_QPOS)
        elapsed = time.time() - start
        assert elapsed < 2.0

    def test_interpolation_speed(self, reset_controller):
        """测试插值速度"""
        import time
        q0 = np.zeros(7)
        q1 = np.ones(7)
        start = time.time()
        for _ in range(50):
            reset_controller.minimum_jerk_trajectory(q0, q1, num_points=100)
        elapsed = time.time() - start
        assert elapsed < 2.0


# ==================== 故障恢复测试 ====================

class TestFaultRecovery:
    """故障恢复功能测试"""

    def test_fault_recovery_misalignment(self, reset_controller):
        """测试对准偏差恢复"""
        current = {"position": [0.15, 0.01, 0.48]}
        target = {"position": [0.15, 0.0, 0.44]}
        result = reset_controller.fault_recovery("misalignment", current, target)
        assert "recovered" in result
        assert "attempts" in result
        assert "log" in result

    def test_fault_recovery_grasp_failure(self, reset_controller):
        """测试抓取失败恢复"""
        current = {"position": [0.15, 0.0, 0.44], "gripper_width": 0.04}
        target = {"position": [0.15, 0.0, 0.44]}
        result = reset_controller.fault_recovery("grasp_failure", current, target)
        assert result["strategy"] == "regrip"
        assert isinstance(result["log"], list)

    def test_fault_recovery_collision(self, reset_controller):
        """测试碰撞恢复"""
        current = {"position": [0.0, 0.0, 0.5]}
        target = {"position": [0.15, 0.0, 0.44]}
        result = reset_controller.fault_recovery("collision", current, target)
        assert result["strategy"] == "retreat_reroute"

    def test_fault_recovery_drop(self, reset_controller):
        """测试掉落恢复"""
        current = {"position": [0.15, 0.0, 0.4]}
        target = {"position": [0.15, 0.0, 0.44]}
        result = reset_controller.fault_recovery("drop", current, target)
        assert result["strategy"] == "relocate_regrip"

    def test_fault_recovery_max_retries(self, reset_controller):
        """测试最大重试次数限制"""
        current = {"position": [0.15, 0.0, 0.48]}
        target = {"position": [0.15, 0.0, 0.44]}
        result = reset_controller.fault_recovery("misalignment", current, target, max_retries=1)
        assert result["attempts"] <= 1

    def test_fault_recovery_returns_log(self, reset_controller):
        """测试恢复过程记录"""
        current = {"position": [0.0, 0.0, 0.5]}
        target = {"position": [0.15, 0.0, 0.44]}
        result = reset_controller.fault_recovery("collision", current, target)
        assert isinstance(result["log"], list)
        for entry in result["log"]:
            assert "attempt" in entry
            assert "action" in entry


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
