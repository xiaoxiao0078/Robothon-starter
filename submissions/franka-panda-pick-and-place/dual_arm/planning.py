"""
Trajectory planning module for Space Module Dual-Arm Assembly.
"""

import numpy as np
from typing import List
from .models import CartesianPose, TrajectoryPoint


class TrajectoryPlanner:
    """轨迹规划器"""
    
    def __init__(self):
        pass
    
    def linear_interpolation_joint(self, q_start: np.ndarray, 
                                   q_end: np.ndarray, 
                                   num_points: int = 50) -> List[TrajectoryPoint]:
        """关节空间线性插值"""
        trajectories = []
        for i in range(num_points):
            t = i / (num_points - 1)
            q = q_start + t * (q_end - q_start)
            qvel = (q_end - q_start) / (num_points - 1)
            qacc = np.zeros_like(q)
            
            trajectories.append(TrajectoryPoint(
                time=t,
                position=q,
                velocity=qvel,
                acceleration=qacc
            ))
        
        return trajectories
    
    def linear_interpolation_cartesian(self, pose_start: CartesianPose,
                                       pose_end: CartesianPose,
                                       num_points: int = 50) -> List[CartesianPose]:
        """笛卡尔空间线性插值"""
        poses = []
        for i in range(num_points):
            t = i / (num_points - 1)
            
            # 位置插值
            pos = pose_start.position + t * (pose_end.position - pose_start.position)
            
            # 姿态插值（SLERP）
            quat = self._slerp(pose_start.orientation, pose_end.orientation, t)
            
            poses.append(CartesianPose(position=pos, orientation=quat))
        
        return poses
    
    def minimum_jerk_trajectory(self, q_start: np.ndarray,
                                q_end: np.ndarray,
                                duration: float = 2.0,
                                num_points: int = 100) -> List[TrajectoryPoint]:
        """最小抖动轨迹"""
        trajectories = []
        
        for i in range(num_points):
            t = i / (num_points - 1)
            tau = t / duration
            
            # 最小抖动多项式
            s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
            ds = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / duration
            dds = (60 * tau - 180 * tau**2 + 120 * tau**3) / duration**2
            
            q = q_start + s * (q_end - q_start)
            qvel = ds * (q_end - q_start)
            qacc = dds * (q_end - q_start)
            
            trajectories.append(TrajectoryPoint(
                time=t * duration,
                position=q,
                velocity=qvel,
                acceleration=qacc
            ))
        
        return trajectories
    
    def _slerp(self, q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
        """球面线性插值"""
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
        
        result = w0 * q0 + w1 * q1
        return result / np.linalg.norm(result)
