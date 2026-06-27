"""
Sensors module for Space Module Dual-Arm Assembly.
"""

import numpy as np
from typing import Dict


class SensorManager:
    """传感器管理器"""
    
    def __init__(self, model, data):
        self.model = model
        self.data = data
    
    def force_estimation(self) -> Dict:
        """力估计"""
        import mujoco
        
        # 获取接触信息
        num_contacts = self.data.ncon
        total_force = np.zeros(3)
        
        for i in range(num_contacts):
            contact = self.data.contact[i]
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            total_force[:3] += force[:3]
        
        force_magnitude = np.linalg.norm(total_force)
        
        return {
            "num_contacts": num_contacts,
            "total_force": total_force.tolist(),
            "force_magnitude": force_magnitude
        }
    
    def collision_detection(self) -> Dict:
        """碰撞检测"""
        num_contacts = self.data.ncon
        has_collision = num_contacts > 0
        
        return {
            "has_collision": has_collision,
            "num_contacts": num_contacts
        }
    
    def get_joint_info(self) -> Dict:
        """获取关节信息"""
        joint_names = []
        for i in range(min(7, self.model.njnt)):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                joint_names.append(name)
            else:
                joint_names.append(f"joint_{i}")
        
        return {
            "num_joints": self.model.njnt,
            "num_dof": self.model.nv,
            "joint_names": joint_names,
            "joint_limits_low": self.model.jnt_range[:7, 0].tolist(),
            "joint_limits_high": self.model.jnt_range[:7, 1].tolist()
        }
    
    def get_end_effector_pos(self) -> np.ndarray:
        """获取末端执行器位置"""
        import mujoco
        
        ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'ee_site')
        if ee_site_id >= 0:
            return self.data.site_xpos[ee_site_id].copy()
        
        ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'hand')
        if ee_body_id >= 0:
            return self.data.xpos[ee_body_id].copy()
        
        return np.zeros(3)
    
    def record_trajectory(self, qpos_list: list, record_force: bool = True) -> list:
        """记录轨迹"""
        import mujoco
        
        recorded = []
        
        for qpos in qpos_list:
            # 设置关节位置
            self.data.qpos[:7] = qpos
            mujoco.mj_forward(self.model, self.data)
            
            # 记录数据
            entry = {
                "qpos": qpos.tolist(),
                "qvel": self.data.qvel[:7].tolist(),
                "time": self.data.time
            }
            
            if record_force:
                force_data = self.force_estimation()
                entry["end_effector_force"] = force_data["force_magnitude"]
            
            recorded.append(entry)
        
        return recorded
