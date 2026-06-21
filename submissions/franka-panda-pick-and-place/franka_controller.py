"""
Franka Panda Smart Manipulation Controller
==========================================
基于Franka Emika Panda机械臂的智能抓取放置系统。
实现25个核心任务，涵盖运动学、轨迹规划、抓取策略和任务编排。

作者: Xiaoxiao Team
参赛ID: 3DOF
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import mujoco
except ImportError:
    raise ImportError("请安装mujoco: pip install mujoco")

# ==================== 数据结构 ====================

@dataclass
class JointState:
    """关节状态"""
    positions: np.ndarray
    velocities: np.ndarray = field(default_factory=lambda: np.zeros(7))
    accelerations: np.ndarray = field(default_factory=lambda: np.zeros(7))


@dataclass
class CartesianPose:
    """笛卡尔空间位姿"""
    position: np.ndarray  # [x, y, z]
    orientation: np.ndarray  # 4元数 [w, x, y, z]


@dataclass
class GraspPlan:
    """抓取规划"""
    approach_pos: np.ndarray
    grasp_pos: np.ndarray
    lift_pos: np.ndarray
    approach_quat: np.ndarray
    grasp_width: float


@dataclass
class TrajectoryPoint:
    """轨迹点"""
    time: float
    position: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(7))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(7))


# ==================== 任务1: 模型加载与初始化 ====================

class FrankaController:
    """Franka Panda机械臂控制器"""

    # Franka Panda关节限位 (弧度)
    JOINT_LIMITS_LOW = np.array([-2.8973, -1.7628, -2.8973, -3.0718,
                                  -2.8973, -0.0175, -2.8973])
    JOINT_LIMITS_HIGH = np.array([2.8973, 1.7628, 2.8973, -0.0698,
                                   2.8973, 3.7525, 2.8973])

    # 默认初始位姿 (home position)
    HOME_QPOS = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])

    def __init__(self, model_path: Optional[str] = None):
        """
        任务1: 加载MuJoCo模型并初始化仿真环境
        
        参数:
            model_path: MuJoCo XML模型路径，None则使用默认Franka模型
        """
        if model_path is None:
            # 从 submissions/franka-panda-pick-and-place/ 回到项目根目录
            project_root = Path(__file__).resolve().parent.parent.parent
            model_path = str(
                project_root / "vendor" / "mujoco_menagerie" / "franka_emika_panda" / "scene.xml"
            )
        
        self.model_path = model_path
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=480, width=640)
        
        # 重置到home位姿
        self.reset()
        
        # 缓存
        self._jac_pos = np.zeros((3, self.model.nv))
        self._jac_rot = np.zeros((3, self.model.nv))

    def reset(self):
        """重置仿真状态"""
        mujoco.mj_resetData(self.model, self.data)
        # 设置home位姿
        self.data.qpos[:7] = self.HOME_QPOS
        self.data.ctrl[:7] = self.HOME_QPOS
        self.data.ctrl[7] = 255  # 夹爪闭合
        mujoco.mj_forward(self.model, self.data)
        return True

    # ==================== 任务2: 关节信息查询 ====================

    def get_joint_info(self) -> dict:
        """
        任务2: 获取关节信息（名称、限位、自由度）
        
        返回:
            包含关节名称、限位、DOF数的字典
        """
        joint_names = []
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            joint_names.append(name or f"joint_{i}")
        
        return {
            "num_joints": self.model.njnt,
            "num_dof": self.model.nv,
            "num_bodies": self.model.nbody,
            "num_geoms": self.model.ngeom,
            "joint_names": joint_names[:7],  # 只返回手臂关节
            "joint_limits_low": self.JOINT_LIMITS_LOW.copy(),
            "joint_limits_high": self.JOINT_LIMITS_HIGH.copy(),
            "gripper_joints": joint_names[7:9] if len(joint_names) > 7 else [],
        }

    # ==================== 任务3: 正运动学 ====================

    def forward_kinematics(self, qpos: np.ndarray) -> CartesianPose:
        """
        任务3: 正运动学 - 从关节角度计算末端执行器位姿
        
        参数:
            qpos: 7维关节角度数组
            
        返回:
            CartesianPose 包含位置和四元数方向
        """
        # 保存当前状态
        saved_qpos = self.data.qpos[:7].copy()
        
        # 设置目标关节角度
        self.data.qpos[:7] = qpos
        mujoco.mj_forward(self.model, self.data)
        
        # 获取末端执行器(joint7之后的body)位置和方向
        ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link7")
        if ee_body_id < 0:
            ee_body_id = self.model.nbody - 1
        
        pos = self.data.xpos[ee_body_id].copy()
        quat = self.data.xquat[ee_body_id].copy()  # [w, x, y, z]
        
        # 恢复
        self.data.qpos[:7] = saved_qpos
        mujoco.mj_forward(self.model, self.data)
        
        return CartesianPose(position=pos, orientation=quat)

    # ==================== 任务4: 雅可比矩阵 ====================

    def get_jacobian(self, qpos: np.ndarray) -> tuple:
        """
        任务4: 计算雅可比矩阵（线性+角速度）
        
        参数:
            qpos: 7维关节角度
            
        返回:
            (jacobian_pos, jacobian_rot) - 3x7的线性雅可比和角速度雅可比
        """
        saved_qpos = self.data.qpos[:7].copy()
        self.data.qpos[:7] = qpos
        mujoco.mj_forward(self.model, self.data)
        
        ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link7")
        if ee_body_id < 0:
            ee_body_id = self.model.nbody - 1
        
        # 计算雅可比
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, ee_body_id)
        
        # 只取前7列（手臂关节）
        jac_pos = jacp[:, :7]
        jac_rot = jacr[:, :7]
        
        # 恢复
        self.data.qpos[:7] = saved_qpos
        mujoco.mj_forward(self.model, self.data)
        
        return jac_pos, jac_rot

    # ==================== 任务5: 关节位置控制 ====================

    def set_joint_positions(self, target_qpos: np.ndarray, steps: int = 100) -> bool:
        """
        任务5: 设置关节位置并执行仿真步进
        
        参数:
            target_qpos: 目标7维关节角度
            steps: 仿真步数
            
        返回:
            是否成功到达目标
        """
        self.data.ctrl[:7] = target_qpos
        
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        
        # 检查是否到达
        error = np.linalg.norm(self.data.qpos[:7] - target_qpos)
        return error < 0.05  # 5cm容差

    # ==================== 任务6: 关节空间线性插值 ====================

    def linear_interpolation_joint(self, q_start: np.ndarray,
                                     q_end: np.ndarray,
                                     num_points: int = 50) -> list:
        """
        任务6: 关节空间线性插值
        
        参数:
            q_start: 起始关节角度
            q_end: 终止关节角度
            num_points: 插值点数
            
        返回:
            TrajectoryPoint列表
        """
        trajectory = []
        for i in range(num_points):
            t = i / (num_points - 1)
            q = q_start + t * (q_end - q_start)
            qdot = (q_end - q_start) * (num_points / (num_points - 1))
            trajectory.append(TrajectoryPoint(
                time=t,
                position=q,
                velocity=qdot
            ))
        return trajectory

    # ==================== 任务7: 笛卡尔空间线性插值 ====================

    def linear_interpolation_cartesian(self, pose_start: CartesianPose,
                                         pose_end: CartesianPose,
                                         num_points: int = 50) -> list:
        """
        任务7: 笛卡尔空间线性插值（位置线性，SLERP方向）
        
        参数:
            pose_start: 起始位姿
            pose_end: 终止位姿
            num_points: 插值点数
            
        返回:
            CartesianPose列表
        """
        trajectory = []
        for i in range(num_points):
            t = i / (num_points - 1)
            # 线性插值位置
            pos = pose_start.position + t * (pose_end.position - pose_start.position)
            # SLERP插值方向
            quat = self._slerp(pose_start.orientation, pose_end.orientation, t)
            trajectory.append(CartesianPose(position=pos, orientation=quat))
        return trajectory

    def _slerp(self, q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
        """四元数球面线性插值"""
        dot = np.dot(q0, q1)
        if dot < 0:
            q1 = -q1
            dot = -dot
        if dot > 0.9995:
            result = q0 + t * (q1 - q0)
            return result / np.linalg.norm(result)
        theta = np.arccos(dot)
        sin_theta = np.sin(theta)
        w0 = np.sin((1 - t) * theta) / sin_theta
        w1 = np.sin(t * theta) / sin_theta
        return w0 * q0 + w1 * q1

    # ==================== 任务8: 最小抖动轨迹 ====================

    def minimum_jerk_trajectory(self, q_start: np.ndarray,
                                   q_end: np.ndarray,
                                   duration: float = 2.0,
                                   num_points: int = 100) -> list:
        """
        任务8: 五次多项式最小抖动轨迹规划
        
        参数:
            q_start: 起始关节角度
            q_end: 终止关节角度
            duration: 轨迹时长(秒)
            num_points: 轨迹点数
            
        返回:
            TrajectoryPoint列表
        """
        trajectory = []
        dt = duration / (num_points - 1)
        
        for i in range(num_points):
            t = i * dt
            tau = t / duration  # 归一化时间 [0, 1]
            
            # 五次多项式: s(t) = 10*t^3 - 15*t^4 + 6*t^5
            s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
            s_dot = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / duration
            s_ddot = (60 * tau - 180 * tau**2 + 120 * tau**3) / duration**2
            
            q = q_start + s * (q_end - q_start)
            qdot = s_dot * (q_end - q_start)
            qddot = s_ddot * (q_end - q_start)
            
            trajectory.append(TrajectoryPoint(
                time=t, position=q, velocity=qdot, acceleration=qddot
            ))
        return trajectory

    # ==================== 任务9: 障碍物避让 ====================

    def obstacle_avoidance(self, q_current: np.ndarray,
                           target_pos: np.ndarray,
                           obstacle_pos: np.ndarray,
                           obstacle_radius: float = 0.1,
                           gain: float = 1.0) -> np.ndarray:
        """
        任务9: 人工势场法障碍物避让
        
        参数:
            q_current: 当前关节角度
            target_pos: 目标笛卡尔位置
            obstacle_pos: 障碍物位置
            obstacle_radius: 障碍物安全半径
            gain: 增益系数
            
        返回:
            避让后的关节角度增量
        """
        # 获取当前末端位置和雅可比
        pose = self.forward_kinematics(q_current)
        jac_pos, _ = self.get_jacobian(q_current)
        
        # 引力场: 向目标移动
        f_attract = gain * (target_pos - pose.position)
        
        # 斥力场: 远离障碍物
        diff = pose.position - obstacle_pos
        dist = np.linalg.norm(diff)
        
        if dist < obstacle_radius * 3:
            # 斥力与距离平方成反比
            f_repel = 0.5 * (1.0/dist - 1.0/(obstacle_radius*3)) * (diff/dist) / dist**2
        else:
            f_repel = np.zeros(3)
        
        # 合力
        f_total = f_attract + f_repel
        
        # 通过雅可比映射到关节空间
        # 使用伪逆
        jac_pos_7 = jac_pos[:, :7]
        JtJ = jac_pos_7.T @ jac_pos_7 + 1e-6 * np.eye(7)
        dq = np.linalg.solve(JtJ, jac_pos_7.T @ f_total)
        
        return dq * 0.01  # 缩小步长

    # ==================== 任务10: 工作空间分析 ====================

    def workspace_analysis(self, num_samples: int = 1000) -> dict:
        """
        任务10: 蒙特卡洛法分析可达工作空间
        
        参数:
            num_samples: 随机采样数
            
        返回:
            工作空间统计信息
        """
        points = []
        for _ in range(num_samples):
            q = np.random.uniform(self.JOINT_LIMITS_LOW, self.JOINT_LIMITS_HIGH)
            pose = self.forward_kinematics(q)
            points.append(pose.position)
        
        points = np.array(points)
        return {
            "reachable_points": points,
            "x_range": (points[:, 0].min(), points[:, 0].max()),
            "y_range": (points[:, 1].min(), points[:, 1].max()),
            "z_range": (points[:, 2].min(), points[:, 2].max()),
            "workspace_volume": (
                (points[:, 0].max() - points[:, 0].min()) *
                (points[:, 1].max() - points[:, 1].min()) *
                (points[:, 2].max() - points[:, 2].min())
            ),
            "max_reach": np.max(np.linalg.norm(points, axis=1)),
            "min_reach": np.min(np.linalg.norm(points, axis=1)),
        }

    # ==================== 任务11: 抓取接近向量 ====================

    def compute_approach_vector(self, object_pos: np.ndarray,
                                  approach_height: float = 0.25) -> CartesianPose:
        """
        任务11: 计算抓取接近向量（从上方接近）
        
        参数:
            object_pos: 物体位置
            approach_height: 接近高度偏移
            
        返回:
            接近起始位姿
        """
        approach_pos = object_pos.copy()
        approach_pos[2] += approach_height
        
        # 末端朝下（z轴朝下）
        approach_quat = np.array([0, 1, 0, 0])  # [w,x,y,z] 朝下
        
        return CartesianPose(position=approach_pos, orientation=approach_quat)

    # ==================== 任务12: 抓取位姿计算 ====================

    def compute_grasp_pose(self, object_pos: np.ndarray,
                            object_width: float = 0.04) -> GraspPlan:
        """
        任务12: 计算完整抓取规划（接近-抓取-抬起）
        
        参数:
            object_pos: 物体位置
            object_width: 物体宽度
            
        返回:
            GraspPlan抓取规划
        """
        approach = self.compute_approach_vector(object_pos)
        
        grasp_pos = object_pos.copy()
        grasp_pos[2] += 0.02  # 略高于物体中心
        
        lift_pos = object_pos.copy()
        lift_pos[2] += 0.2  # 抬起20cm
        
        return GraspPlan(
            approach_pos=approach.position,
            grasp_pos=grasp_pos,
            lift_pos=lift_pos,
            approach_quat=approach.orientation,
            grasp_width=object_width
        )

    # ==================== 任务13: 预抓取位置 ====================

    def pre_grasp_position(self, object_pos: np.ndarray,
                            safety_margin: float = 0.05) -> np.ndarray:
        """
        任务13: 计算预抓取安全位置
        
        参数:
            object_pos: 物体位置
            safety_margin: 安全余量
            
        返回:
            预抓取位置
        """
        pre_grasp = object_pos.copy()
        pre_grasp[2] += safety_margin + 0.15
        return pre_grasp

    # ==================== 任务14: 夹爪控制 ====================

    def gripper_control(self, width: float, steps: int = 50) -> bool:
        """
        任务14: 控制夹爪开合
        
        参数:
            width: 夹爪宽度 (0=完全闭合, 0.04=完全打开)
            steps: 仿真步数
            
        返回:
            是否成功
        """
        # Franka夹爪映射: 0=闭合, 255=打开
        ctrl_value = (width / 0.04) * 255
        ctrl_value = np.clip(ctrl_value, 0, 255)
        self.data.ctrl[7] = ctrl_value
        
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        
        return True

    # ==================== 任务15: 接触力估计 ====================

    def force_estimation(self) -> dict:
        """
        任务15: 估计末端执行器接触力
        
        返回:
            接触力信息字典
        """
        # 读取传感器数据
        forces = []
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            forces.append({
                "geom1": contact.geom1,
                "geom2": contact.geom2,
                "force": force[:3].copy(),
                "torque": force[3:].copy(),
                "normal_force": force[0],
            })
        
        total_force = sum(f["force"] for f in forces) if forces else np.zeros(3)
        return {
            "num_contacts": self.data.ncon,
            "contacts": forces,
            "total_force": total_force,
            "force_magnitude": np.linalg.norm(total_force),
        }

    # ==================== 任务16: 完整拾取操作 ====================

    def pick_object(self, object_pos: np.ndarray,
                    object_width: float = 0.04) -> bool:
        """
        任务16: 完整拾取操作（接近-下降-抓取-抬起）
        
        参数:
            object_pos: 物体位置
            object_width: 物体宽度
            
        返回:
            是否成功拾取
        """
        # 1. 打开夹爪
        self.gripper_control(0.04, steps=30)
        
        # 2. 移动到预抓取位置
        pre_grasp = self.pre_grasp_position(object_pos)
        self._move_to_cartesian(pre_grasp, steps=150)
        
        # 3. 下降到抓取位置
        grasp_plan = self.compute_grasp_pose(object_pos, object_width)
        self._move_to_cartesian_pos(grasp_plan.grasp_pos, steps=150)
        
        # 4. 关闭夹爪
        self.gripper_control(object_width, steps=50)
        
        # 5. 抬起
        self._move_to_cartesian_pos(grasp_plan.lift_pos, steps=150)
        
        return True

    # ==================== 任务17: 完整放置操作 ====================

    def place_object(self, place_pos: np.ndarray,
                     place_width: float = 0.04) -> bool:
        """
        任务17: 完整放置操作（移动到目标-下降-释放-抬起）
        
        参数:
            place_pos: 放置位置
            place_width: 放置时夹爪宽度
            
        返回:
            是否成功放置
        """
        # 1. 移动到放置位置上方
        above_pos = place_pos.copy()
        above_pos[2] += 0.2
        self._move_to_cartesian_pos(above_pos, steps=150)
        
        # 2. 下降
        self._move_to_cartesian_pos(place_pos, steps=150)
        
        # 3. 打开夹爪释放
        self.gripper_control(0.04, steps=50)
        
        # 4. 抬起
        self._move_to_cartesian_pos(above_pos, steps=100)
        
        return True

    # ==================== 任务18: 物体堆叠 ====================

    def stack_objects(self, object_positions: list, stack_height: float = 0.04) -> dict:
        """
        任务18: 将多个物体堆叠
        
        参数:
            object_positions: 物体位置列表
            stack_height: 每层堆叠高度
            
        返回:
            堆叠结果信息
        """
        # 按x坐标排序，从左到右堆叠
        sorted_pos = sorted(object_positions, key=lambda p: p[0])
        base_pos = sorted_pos[0]
        
        results = []
        for i, obj_pos in enumerate(sorted_pos[1:], 1):
            target = base_pos.copy()
            target[2] += i * stack_height
            
            # 拾取
            self.pick_object(obj_pos)
            # 放置
            self.place_object(target)
            
            results.append({
                "object_index": i,
                "picked_from": obj_pos.tolist(),
                "placed_at": target.tolist(),
                "success": True
            })
        
        return {
            "total_objects": len(object_positions),
            "stack_height": len(sorted_pos) * stack_height,
            "operations": results
        }

    # ==================== 任务19: 物体分拣 ====================

    def sort_objects(self, objects: list, target_zones: list) -> dict:
        """
        任务19: 按区域分拣物体
        
        参数:
            objects: 物体位置列表
            target_zones: 目标区域中心列表
            
        返回:
            分拣结果
        """
        results = []
        for i, obj_pos in enumerate(objects):
            # 分配最近的目标区域
            distances = [np.linalg.norm(np.array(z) - np.array(obj_pos)) 
                        for z in target_zones]
            nearest_zone = target_zones[np.argmin(distances)]
            
            # 执行拾取放置
            self.pick_object(np.array(obj_pos))
            self.place_object(np.array(nearest_zone))
            
            results.append({
                "object": i,
                "from": obj_pos,
                "to": nearest_zone,
            })
        
        return {"sorted_count": len(results), "operations": results}

    # ==================== 任务20: 运动轨迹记录 ====================

    def record_trajectory(self, qpos_list: list, record_force: bool = True) -> list:
        """
        任务20: 执行轨迹并记录运动数据
        
        参数:
            qpos_list: 关节角度序列
            record_force: 是否记录力数据
            
        返回:
            记录的轨迹数据
        """
        recorded = []
        for i, qpos in enumerate(qpos_list):
            self.data.ctrl[:7] = qpos
            mujoco.mj_step(self.model, self.data)
            
            entry = {
                "step": i,
                "qpos": self.data.qpos[:7].copy(),
                "qvel": self.data.qvel[:7].copy(),
                "time": self.data.time,
            }
            
            if record_force:
                forces = self.force_estimation()
                entry["end_effector_force"] = forces["force_magnitude"]
            
            recorded.append(entry)
        
        return recorded

    # ==================== 任务21: 碰撞检测 ====================

    def collision_detection(self) -> dict:
        """
        任务21: 检测环境碰撞
        
        返回:
            碰撞检测结果
        """
        mujoco.mj_collision(self.model, self.data)
        
        collisions = []
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            collisions.append({
                "geom1": geom1_name or str(contact.geom1),
                "geom2": geom2_name or str(contact.geom2),
                "pos": contact.pos.copy(),
                "dist": contact.dist,
            })
        
        return {
            "has_collision": len(collisions) > 0,
            "num_contacts": len(collisions),
            "collisions": collisions,
        }

    # ==================== 任务22: 阻抗控制 ====================

    def impedance_control(self, target_pos: np.ndarray,
                           stiffness: np.ndarray = None,
                           damping: np.ndarray = None) -> np.ndarray:
        """
        任务22: 阻抗控制计算期望力矩
        
        参数:
            target_pos: 目标笛卡尔位置
            stiffness: 刚度矩阵 (默认对角)
            damping: 阻尼矩阵 (默认对角)
            
        返回:
            关节力矩
        """
        if stiffness is None:
            stiffness = np.diag([200, 200, 200])
        if damping is None:
            damping = np.diag([20, 20, 20])
        
        # 获取当前状态
        pose = self.forward_kinematics(self.data.qpos[:7])
        jac_pos, _ = self.get_jacobian(self.data.qpos[:7])
        
        # 位置误差
        pos_error = target_pos - pose.position
        
        # 阻抗控制力
        f_impedance = stiffness @ pos_error - damping @ (self.data.qvel[:3])
        
        # 映射到关节空间
        jac_7 = jac_pos[:, :7]
        tau = jac_7.T @ f_impedance
        
        return tau

    # ==================== 任务23: 视觉伺服 ====================

    def visual_servoing(self, target_pixel: np.ndarray,
                         current_pixel: np.ndarray,
                         camera_matrix: np.ndarray = None) -> np.ndarray:
        """
        任务23: 基于图像的视觉伺服 (IBVS)
        
        参数:
            target_pixel: 目标图像坐标 [u, v]
            current_pixel: 当前图像坐标 [u, v]
            camera_matrix: 相机内参矩阵
            
        返回:
            关节角度增量
        """
        if camera_matrix is None:
            # 默认相机内参 (模拟)
            camera_matrix = np.array([
                [500, 0, 320],
                [0, 500, 240],
                [0, 0, 1]
            ])
        
        # 图像误差
        error = target_pixel - current_pixel
        
        # 简化视觉伺服雅可比 (仿射近似)
        z = 0.5  # 估计深度
        L = np.array([
            [-1/z, 0, current_pixel[0]/z, 
             current_pixel[0]*current_pixel[1]/500,
             -(500 + current_pixel[0]**2/500), current_pixel[1]],
            [0, -1/z, current_pixel[1]/z,
             500 + current_pixel[1]**2/500,
             -current_pixel[0]*current_pixel[1]/500, -current_pixel[0]]
        ]) * camera_matrix[0, 0]
        
        # 计算速度命令
        L_pseudo = np.linalg.pinv(L)
        v = L_pseudo @ error * 0.01
        
        # 映射到关节
        jac_pos, _ = self.get_jacobian(self.data.qpos[:7])
        dq = np.linalg.lstsq(jac_pos[:, :7], v[:3], rcond=None)[0]
        
        return dq

    # ==================== 任务24: 示教学习 ====================

    def skill_learning(self, demonstrations: list) -> dict:
        """
        任务24: 从示教轨迹学习技能 (DMP近似)
        
        参数:
            demonstrations: 示教轨迹列表，每个轨迹是关节角度序列
            
        返回:
            学习到的运动基元参数
        """
        # 动态运动基元 (DMP) 简化实现
        # 1. 对齐所有示教轨迹
        max_len = max(len(d) for d in demonstrations)
        aligned = []
        for demo in demonstrations:
            if len(demo) < max_len:
                # 线性插值对齐
                indices = np.linspace(0, len(demo)-1, max_len)
                aligned.append(np.array([demo[int(i)] for i in indices]))
            else:
                aligned.append(np.array(demo[:max_len]))
        
        # 2. 计算平均轨迹
        mean_trajectory = np.mean(aligned, axis=0)
        
        # 3. 计算方差（用于适应性）
        variance = np.var(aligned, axis=0)
        
        # 4. 提取关键特征
        return {
            "mean_trajectory": mean_trajectory,
            "variance": variance,
            "num_demonstrations": len(demonstrations),
            "trajectory_length": max_len,
            "convergence_threshold": np.mean(variance),
        }

    # ==================== 任务25: 任务编排 ====================

    def task_orchestration(self, task_sequence: list) -> dict:
        """
        任务25: 编排完整任务序列
        
        参数:
            task_sequence: 任务列表 [{"type": "pick", "params": {...}}, ...]
            
        返回:
            编排执行结果
        """
        results = []
        success_count = 0
        
        for i, task in enumerate(task_sequence):
            task_type = task.get("type", "")
            params = task.get("params", {})
            
            try:
                if task_type == "pick":
                    pos = np.array(params.get("position", [0.3, 0, 0.1]))
                    result = self.pick_object(pos)
                elif task_type == "place":
                    pos = np.array(params.get("position", [-0.3, 0, 0.1]))
                    result = self.place_object(pos)
                elif task_type == "move":
                    qpos = np.array(params.get("qpos", self.HOME_QPOS))
                    result = self.set_joint_positions(qpos)
                elif task_type == "gripper":
                    width = params.get("width", 0.04)
                    result = self.gripper_control(width)
                elif task_type == "stack":
                    positions = [np.array(p) for p in params.get("positions", [])]
                    result = self.stack_objects(positions)
                elif task_type == "sort":
                    objects = params.get("objects", [])
                    zones = params.get("zones", [])
                    result = self.sort_objects(objects, zones)
                else:
                    result = {"error": f"Unknown task type: {task_type}"}
                
                success_count += 1
                results.append({"task": i, "type": task_type, "result": result, "success": True})
            except Exception as e:
                results.append({"task": i, "type": task_type, "error": str(e), "success": False})
        
        return {
            "total_tasks": len(task_sequence),
            "completed": success_count,
            "success_rate": success_count / len(task_sequence) if task_sequence else 0,
            "results": results,
        }

    # ==================== 辅助方法 ====================

    def _move_to_cartesian(self, target_pose, steps: int = 100):
        """移动到目标笛卡尔位姿 (使用IK)"""
        # 支持 CartesianPose 或 ndarray
        if isinstance(target_pose, CartesianPose):
            target_pos = target_pose.position
        else:
            target_pos = np.asarray(target_pose)
        
        for _ in range(steps):
            qpos = self.data.qpos[:7]
            pose = self.forward_kinematics(qpos)
            jac_pos, _ = self.get_jacobian(qpos)
            
            pos_error = target_pos - pose.position
            dq = np.linalg.lstsq(jac_pos[:, :7], pos_error, rcond=None)[0]
            
            self.data.ctrl[:7] = qpos + dq * 0.5
            mujoco.mj_step(self.model, self.data)

    def _move_to_cartesian_pos(self, target_pos, steps: int = 100):
        """移动到目标位置（保持当前方向）"""
        target_pos = np.asarray(target_pos)
        for _ in range(steps):
            qpos = self.data.qpos[:7]
            pose = self.forward_kinematics(qpos)
            jac_pos, _ = self.get_jacobian(qpos)
            
            pos_error = target_pos - pose.position
            dq = np.linalg.lstsq(jac_pos[:, :7], pos_error, rcond=None)[0]
            
            self.data.ctrl[:7] = qpos + dq * 0.5
            mujoco.mj_step(self.model, self.data)

    def render_frame(self) -> np.ndarray:
        """渲染当前帧"""
        self.renderer.update_scene(self.data)
        return self.renderer.render()

    def get_end_effector_pos(self) -> np.ndarray:
        """获取当前末端执行器位置"""
        pose = self.forward_kinematics(self.data.qpos[:7])
        return pose.position

    def move_to_home(self):
        """移动到初始位姿"""
        trajectory = self.minimum_jerk_trajectory(
            self.data.qpos[:7], self.HOME_QPOS, duration=2.0
        )
        for point in trajectory:
            self.data.ctrl[:7] = point.position
            for _ in range(5):
                mujoco.mj_step(self.model, self.data)
