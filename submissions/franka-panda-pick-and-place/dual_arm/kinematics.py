"""
Kinematics module for Space Module Dual-Arm Assembly.
"""

import numpy as np
from typing import Tuple
from .models import CartesianPose


class Kinematics:
    """运动学计算"""
    
    def __init__(self, model, data):
        self.model = model
        self.data = data
    
    def forward_kinematics(self, qpos: np.ndarray) -> CartesianPose:
        """正运动学"""
        import mujoco
        
        # 设置关节位置
        self.data.qpos[:7] = qpos
        mujoco.mj_forward(self.model, self.data)
        
        # 获取末端执行器位置
        ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'ee_site')
        if ee_site_id >= 0:
            pos = self.data.site_xpos[ee_site_id].copy()
            mat = self.data.site_xmat[ee_site_id].reshape(3, 3).copy()
        else:
            # 如果没有site，使用body
            ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'hand')
            if ee_body_id >= 0:
                pos = self.data.xpos[ee_body_id].copy()
                mat = self.data.xmat[ee_body_id].reshape(3, 3).copy()
            else:
                pos = np.zeros(3)
                mat = np.eye(3)
        
        # 旋转矩阵转四元数
        quat = self._rotation_matrix_to_quaternion(mat)
        
        return CartesianPose(position=pos, orientation=quat)
    
    def get_jacobian(self, qpos: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """计算雅可比矩阵"""
        import mujoco
        
        # 设置关节位置
        self.data.qpos[:7] = qpos
        mujoco.mj_forward(self.model, self.data)
        
        # 计算雅可比
        jac_pos = np.zeros((3, self.model.nv))
        jac_rot = np.zeros((3, self.model.nv))
        
        ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'ee_site')
        if ee_site_id >= 0:
            mujoco.mj_jacSite(self.model, self.data, jac_pos, jac_rot, ee_site_id)
        else:
            ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'hand')
            if ee_body_id >= 0:
                mujoco.mj_jacBody(self.model, self.data, jac_pos, jac_rot, ee_body_id)
        
        # 只返回前7列（7个关节）
        return jac_pos[:, :7], jac_rot[:, :7]
    
    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """旋转矩阵转四元数"""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return np.array([w, x, y, z])
