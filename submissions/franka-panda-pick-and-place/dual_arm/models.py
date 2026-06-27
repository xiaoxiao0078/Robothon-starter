"""
Data models for Space Module Dual-Arm Assembly.
"""

from dataclasses import dataclass
from typing import List
import numpy as np


@dataclass
class JointState:
    """关节状态"""
    position: np.ndarray  # 关节位置 (rad)
    velocity: np.ndarray  # 关节速度 (rad/s)
    effort: np.ndarray    # 关节力矩 (Nm)


@dataclass
class CartesianPose:
    """笛卡尔位姿"""
    position: np.ndarray    # 位置 (x, y, z) in meters
    orientation: np.ndarray # 四元数 (w, x, y, z)


@dataclass
class GraspPlan:
    """抓取规划"""
    approach_pos: np.ndarray  # 接近位置
    grasp_pos: np.ndarray     # 抓取位置
    lift_pos: np.ndarray      # 抬起位置
    grasp_width: float        # 抓取宽度
    grasp_force: float        # 抓取力


@dataclass
class TrajectoryPoint:
    """轨迹点"""
    time: float               # 时间戳
    position: np.ndarray      # 关节位置
    velocity: np.ndarray      # 关节速度
    acceleration: np.ndarray  # 关节加速度
