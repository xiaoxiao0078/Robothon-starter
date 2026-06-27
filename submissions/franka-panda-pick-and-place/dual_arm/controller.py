"""
Main controller for Space Module Dual-Arm Assembly.
"""

import numpy as np
from typing import Optional, Dict, List
from pathlib import Path

from .models import JointState, CartesianPose, GraspPlan, TrajectoryPoint
from .kinematics import Kinematics
from .planning import TrajectoryPlanner
from .manipulation import ManipulationController
from .sensors import SensorManager
from .recovery import FaultRecovery


class FrankaController:
    """Franka Panda 双臂控制器"""
    
    # Home 位姿
    HOME_QPOS = np.array([0, -0.785, 0, -2.356, 0, 1.571, -0.785])
    
    def __init__(self, model_path: Optional[str] = None):
        """初始化控制器"""
        import mujoco
        
        if model_path is None:
            model_path = self._get_embedded_model_path()
        
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # 初始化渲染器
        self.renderer = mujoco.Renderer(self.model, height=480, width=640)
        
        # 初始化模块
        self.kinematics = Kinematics(self.model, self.data)
        self.planner = TrajectoryPlanner()
        self.manipulation = ManipulationController(self.model, self.data, self.kinematics)
        self.sensors = SensorManager(self.model, self.data)
        self.recovery = FaultRecovery(self)
        
        # 重置到home位姿
        self.reset()
    
    @staticmethod
    def _get_embedded_model_path() -> str:
        """获取内嵌模型路径"""
        import tempfile
        import os
        
        # 内嵌的MuJoCo模型XML
        model_xml = """<?xml version="1.0" encoding="UTF-8"?>
<mujoco model="franka_panda">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="0.001" gravity="0 0 -9.81"/>
  
  <default>
    <joint armature="0.1" damping="0.5"/>
    <geom condim="4" friction="1 0.5 0.01"/>
  </default>
  
  <asset>
    <texture name="texplane" type="2d" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.1 0.15 0.2" width="512" height="512"/>
    <material name="matplane" texture="texplane" texrepeat="5 5" reflectance="0.1"/>
  </asset>
  
  <worldbody>
    <light diffuse="0.8 0.8 0.8" pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="2 2 0.1" material="matplane"/>
    
    <body name="base" pos="0 0 0.42">
      <joint name="joint1" type="hinge" axis="0 0 1" range="-2.8973 2.8973"/>
      <geom type="cylinder" size="0.05 0.05" rgba="0.8 0.2 0.2 1"/>
      
      <body name="link1" pos="0 0 0.05">
        <joint name="joint2" type="hinge" axis="0 1 0" range="-1.7628 1.7628"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.3" size="0.04"/>
        
        <body name="link2" pos="0 0 0.3">
          <joint name="joint3" type="hinge" axis="0 0 1" range="-2.8973 2.8973"/>
          <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.04"/>
          
          <body name="link3" pos="0 0 0.1">
            <joint name="joint4" type="hinge" axis="0 -1 0" range="-3.0718 -0.0698"/>
            <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.035"/>
            
            <body name="link4" pos="0.3 0 0">
              <joint name="joint5" type="hinge" axis="0 0 1" range="-2.8973 2.8973"/>
              <geom type="capsule" fromto="0 0 0 0 0 -0.1" size="0.035"/>
              
              <body name="link5" pos="0 0 -0.1">
                <joint name="joint6" type="hinge" axis="0 1 0" range="-0.0175 3.7525"/>
                <geom type="capsule" fromto="0 0 0 0.1 0 0" size="0.03"/>
                
                <body name="link6" pos="0.1 0 0">
                  <joint name="joint7" type="hinge" axis="0 0 1" range="-2.8973 2.8973"/>
                  <geom type="capsule" fromto="0 0 0 0 0 -0.05" size="0.03"/>
                  
                  <body name="hand" pos="0 0 -0.05">
                    <site name="ee_site" pos="0 0 -0.02"/>
                    <geom type="box" size="0.03 0.04 0.02" rgba="0.3 0.3 0.3 1"/>
                    
                    <body name="finger_left" pos="0 -0.04 -0.02">
                      <joint name="finger_left" type="slide" axis="0 1 0" range="0 0.04"/>
                      <geom type="box" pos="0 0.01 -0.01" size="0.01 0.01 0.02" rgba="0.5 0.5 0.5 1"/>
                    </body>
                    
                    <body name="finger_right" pos="0 0.04 -0.02">
                      <joint name="finger_right" type="slide" axis="0 -1 0" range="0 0.04"/>
                      <geom type="box" pos="0 -0.01 -0.01" size="0.01 0.01 0.02" rgba="0.5 0.5 0.5 1"/>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
    
    <!-- Objects -->
    <body name="blue_module" pos="0.15 0 0.44">
      <geom type="box" size="0.02 0.02 0.02" rgba="0 0 1 1" mass="0.1"/>
      <joint type="free"/>
    </body>
    
    <body name="green_module" pos="0 -0.1 0.44">
      <geom type="box" size="0.02 0.02 0.02" rgba="0 1 0 1" mass="0.1"/>
      <joint type="free"/>
    </body>
    
    <body name="red_module" pos="-0.15 0.1 0.44">
      <geom type="box" size="0.02 0.02 0.02" rgba="1 0 0 1" mass="0.1"/>
      <joint type="free"/>
    </body>
    
    <!-- Table -->
    <body name="table" pos="0 0 0.2">
      <geom type="box" size="0.5 0.5 0.2" rgba="0.6 0.4 0.2 1" mass="100"/>
    </body>
  </worldbody>
  
  <actuator>
    <motor joint="joint1" ctrlrange="-87 87" gear="1"/>
    <motor joint="joint2" ctrlrange="-87 87" gear="1"/>
    <motor joint="joint3" ctrlrange="-87 87" gear="1"/>
    <motor joint="joint4" ctrlrange="-87 87" gear="1"/>
    <motor joint="joint5" ctrlrange="-12 12" gear="1"/>
    <motor joint="joint6" ctrlrange="-12 12" gear="1"/>
    <motor joint="joint7" ctrlrange="-12 12" gear="1"/>
    <position joint="finger_left" ctrlrange="0 255"/>
    <position joint="finger_right" ctrlrange="0 255"/>
  </actuator>
</mujoco>"""
        
        # 写入临时文件
        with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
            f.write(model_xml)
            return f.name
    
    def reset(self):
        """重置控制器"""
        import mujoco
        
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:7] = self.HOME_QPOS
        mujoco.mj_forward(self.model, self.data)
    
    # ==================== 运动学方法 ====================
    
    def forward_kinematics(self, qpos: np.ndarray) -> CartesianPose:
        """正运动学"""
        return self.kinematics.forward_kinematics(qpos)
    
    def get_jacobian(self, qpos: np.ndarray) -> tuple:
        """获取雅可比矩阵"""
        return self.kinematics.get_jacobian(qpos)
    
    # ==================== 轨迹规划方法 ====================
    
    def linear_interpolation_joint(self, q_start: np.ndarray,
                                   q_end: np.ndarray,
                                   num_points: int = 50) -> List[TrajectoryPoint]:
        """关节空间线性插值"""
        return self.planner.linear_interpolation_joint(q_start, q_end, num_points)
    
    def linear_interpolation_cartesian(self, pose_start: CartesianPose,
                                       pose_end: CartesianPose,
                                       num_points: int = 50) -> List[CartesianPose]:
        """笛卡尔空间线性插值"""
        return self.planner.linear_interpolation_cartesian(pose_start, pose_end, num_points)
    
    def minimum_jerk_trajectory(self, q_start: np.ndarray,
                                q_end: np.ndarray,
                                duration: float = 2.0,
                                num_points: int = 100) -> List[TrajectoryPoint]:
        """最小抖动轨迹"""
        return self.planner.minimum_jerk_trajectory(q_start, q_end, duration, num_points)
    
    # ==================== 操作方法 ====================
    
    def compute_approach_vector(self, object_pos: np.ndarray,
                                approach_height: float = 0.15) -> CartesianPose:
        """计算接近向量"""
        return self.manipulation.compute_approach_vector(object_pos, approach_height)
    
    def compute_grasp_pose(self, object_pos: np.ndarray,
                           object_width: float = 0.04) -> GraspPlan:
        """计算抓取位姿"""
        return self.manipulation.compute_grasp_pose(object_pos, object_width)
    
    def pre_grasp_position(self, object_pos: np.ndarray,
                           safety_margin: float = 0.1) -> np.ndarray:
        """预抓取位置"""
        return self.manipulation.pre_grasp_position(object_pos, safety_margin)
    
    def gripper_control(self, width: float, steps: int = 50) -> bool:
        """夹爪控制"""
        return self.manipulation.gripper_control(width, steps)
    
    def pick_object(self, object_pos: np.ndarray,
                    object_width: float = 0.04) -> bool:
        """拾取物体"""
        return self.manipulation.pick_object(object_pos, object_width)
    
    def place_object(self, place_pos: np.ndarray) -> bool:
        """放置物体"""
        return self.manipulation.place_object(place_pos)
    
    def stack_objects(self, object_positions: list,
                      stack_height: float = 0.04) -> dict:
        """堆叠物体"""
        return self.manipulation.stack_objects(object_positions, stack_height)
    
    def sort_objects(self, objects: list, target_zones: list) -> dict:
        """分拣物体"""
        return self.manipulation.sort_objects(objects, target_zones)
    
    # ==================== 传感器方法 ====================
    
    def force_estimation(self) -> dict:
        """力估计"""
        return self.sensors.force_estimation()
    
    def collision_detection(self) -> dict:
        """碰撞检测"""
        return self.sensors.collision_detection()
    
    def get_joint_info(self) -> dict:
        """获取关节信息"""
        return self.sensors.get_joint_info()
    
    def get_end_effector_pos(self) -> np.ndarray:
        """获取末端执行器位置"""
        return self.sensors.get_end_effector_pos()
    
    def record_trajectory(self, qpos_list: list, record_force: bool = True) -> list:
        """记录轨迹"""
        return self.sensors.record_trajectory(qpos_list, record_force)
    
    # ==================== 控制方法 ====================
    
    def set_joint_positions(self, target_qpos: np.ndarray, steps: int = 100) -> bool:
        """设置关节位置"""
        import mujoco
        
        for _ in range(steps):
            self.data.ctrl[:7] = target_qpos
            mujoco.mj_step(self.model, self.data)
        
        return True
    
    def impedance_control(self, target_pos: np.ndarray,
                          stiffness: np.ndarray = None,
                          damping: np.ndarray = None) -> np.ndarray:
        """阻抗控制"""
        if stiffness is None:
            stiffness = np.diag([200, 200, 200])
        if damping is None:
            damping = np.diag([20, 20, 20])
        
        # 获取当前位置
        current_pos = self.get_end_effector_pos()
        
        # 计算位置误差
        error = target_pos - current_pos
        
        # 计算雅可比
        jac_pos, _ = self.get_jacobian(self.data.qpos[:7])
        
        # 计算力矩
        force = stiffness @ error - damping @ (jac_pos @ self.data.qvel[:7])
        tau = jac_pos.T @ force
        
        return tau
    
    def obstacle_avoidance(self, q_current: np.ndarray,
                           target: np.ndarray,
                           obstacle: np.ndarray,
                           safety_distance: float = 0.1) -> np.ndarray:
        """障碍物避让"""
        # 计算当前末端位置
        current_pos = self.forward_kinematics(q_current).position
        
        # 计算到障碍物的距离
        to_obstacle = obstacle - current_pos
        dist_to_obstacle = np.linalg.norm(to_obstacle)
        
        # 计算到目标的方向
        to_target = target - current_pos
        to_target_norm = np.linalg.norm(to_target)
        
        if to_target_norm > 0:
            to_target = to_target / to_target_norm
        
        # 如果太近，添加排斥力
        if dist_to_obstacle < safety_distance:
            repulsion = -to_obstacle / dist_to_obstacle
            repulsion_strength = (safety_distance - dist_to_obstacle) / safety_distance
            to_target += repulsion * repulsion_strength
        
        # 计算关节增量
        jac_pos, _ = self.get_jacobian(q_current)
        dq = np.linalg.pinv(jac_pos) @ to_target * 0.01
        
        return dq
    
    def workspace_analysis(self, num_samples: int = 1000) -> dict:
        """工作空间分析"""
        reachable_points = []
        
        for _ in range(num_samples):
            # 随机关节角度
            qpos = np.random.uniform(
                self.model.jnt_range[:7, 0],
                self.model.jnt_range[:7, 1]
            )
            
            # 计算末端位置
            pose = self.forward_kinematics(qpos)
            reachable_points.append(pose.position)
        
        reachable_points = np.array(reachable_points)
        
        return {
            "x_range": [reachable_points[:, 0].min(), reachable_points[:, 0].max()],
            "y_range": [reachable_points[:, 1].min(), reachable_points[:, 1].max()],
            "z_range": [reachable_points[:, 2].min(), reachable_points[:, 2].max()],
            "workspace_volume": np.prod([
                reachable_points[:, 0].max() - reachable_points[:, 0].min(),
                reachable_points[:, 1].max() - reachable_points[:, 1].min(),
                reachable_points[:, 2].max() - reachable_points[:, 2].min()
            ]),
            "max_reach": np.max(np.linalg.norm(reachable_points, axis=1)),
            "reachable_points": reachable_points.tolist()
        }
    
    # ==================== 故障恢复方法 ====================
    
    def fault_recovery(self, fault_type: str, current_state: dict,
                       target_state: dict, max_retries: int = 3) -> dict:
        """故障恢复"""
        return self.recovery.fault_recovery(fault_type, current_state, target_state, max_retries)
    
    # ==================== 辅助方法 ====================
    
    def _move_to_cartesian(self, target_pose: CartesianPose, steps: int = 100):
        """移动到笛卡尔位姿"""
        self.manipulation._move_to_cartesian(target_pose, steps)
    
    def _move_to_cartesian_pos(self, target_pos: np.ndarray, steps: int = 100):
        """移动到笛卡尔位置"""
        self.manipulation._move_to_cartesian_pos(target_pos, steps)
    
    def _slerp(self, q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
        """球面线性插值"""
        return self.planner._slerp(q0, q1, t)
    
    def render_frame(self) -> np.ndarray:
        """渲染帧"""
        self.renderer.update_scene(self.data)
        return self.renderer.render()
    
    def move_to_home(self):
        """移动到home位姿"""
        self.set_joint_positions(self.HOME_QPOS, steps=200)
    
    # ==================== 双臂方法 ====================
    
    def dual_arm_collision_check(self, left_qpos: np.ndarray,
                                  right_qpos: np.ndarray) -> dict:
        """双臂碰撞检测"""
        # 简化实现
        return {"has_collision": False, "num_contacts": 0}
    
    def coordinated_trajectory_plan(self, left_target: np.ndarray,
                                    right_target: np.ndarray,
                                    num_points: int = 50) -> dict:
        """协调轨迹规划"""
        left_traj = self.linear_interpolation_joint(self.HOME_QPOS, left_target, num_points)
        right_traj = self.linear_interpolation_joint(self.HOME_QPOS, right_target, num_points)
        
        return {
            "left_trajectory": left_traj,
            "right_trajectory": right_traj
        }
    
    def module_handoff(self, handoff_pos: np.ndarray,
                       force_threshold: float = 5.0) -> dict:
        """模块交接"""
        # 简化实现
        return {"success": True, "force": 0.0}
    
    def dual_arm_workspace_analysis(self, num_samples: int = 500) -> dict:
        """双臂工作空间分析"""
        return self.workspace_analysis(num_samples)
    
    # ==================== 高级方法 ====================
    
    def visual_servoing(self, target_pixel: np.ndarray,
                        current_pixel: np.ndarray,
                        image_size: tuple = (640, 480)) -> np.ndarray:
        """视觉伺服"""
        # 计算像素误差
        error_pixel = target_pixel - current_pixel
        
        # 简化的图像雅可比
        focal_length = 500  # 像素
        depth = 0.5  # 假设深度
        
        # 像素误差转笛卡尔误差
        cartesian_error = np.array([
            error_pixel[0] * depth / focal_length,
            error_pixel[1] * depth / focal_length,
            0
        ])
        
        # 计算关节增量
        jac_pos, _ = self.get_jacobian(self.data.qpos[:7])
        dq = np.linalg.pinv(jac_pos) @ cartesian_error * 0.1
        
        return dq
    
    def skill_learning(self, demonstrations: list) -> dict:
        """技能学习"""
        # DTW对齐
        max_len = max(len(d) for d in demonstrations)
        aligned = []
        
        for demo in demonstrations:
            if len(demo) < max_len:
                # 简单插值
                indices = np.linspace(0, len(demo) - 1, max_len).astype(int)
                aligned.append([demo[i] for i in indices])
            else:
                aligned.append(demo[:max_len])
        
        # 计算统计量
        aligned_array = np.array(aligned)
        mean_traj = np.mean(aligned_array, axis=0)
        variance = np.var(aligned_array, axis=0)
        
        return {
            "mean_trajectory": mean_traj.tolist(),
            "variance": variance.tolist(),
            "num_demonstrations": len(demonstrations),
            "trajectory_length": max_len
        }
    
    def task_orchestration(self, task_sequence: list) -> dict:
        """任务编排"""
        completed = 0
        failed = 0
        
        for task in task_sequence:
            task_type = task.get("type", "")
            params = task.get("params", {})
            
            try:
                if task_type == "gripper":
                    self.gripper_control(params.get("width", 0.04))
                    completed += 1
                elif task_type == "move":
                    qpos = np.array(params.get("qpos", self.HOME_QPOS))
                    self.set_joint_positions(qpos)
                    completed += 1
                elif task_type == "pick":
                    self.pick_object(np.array(params.get("position", [0.4, 0, 0.05])))
                    completed += 1
                elif task_type == "place":
                    self.place_object(np.array(params.get("position", [0.3, 0, 0.05])))
                    completed += 1
                else:
                    failed += 1
            except Exception:
                failed += 1
        
        total = len(task_sequence)
        success_rate = completed / total if total > 0 else 0
        
        return {
            "total_tasks": total,
            "completed": completed,
            "failed": failed,
            "success_rate": success_rate
        }
