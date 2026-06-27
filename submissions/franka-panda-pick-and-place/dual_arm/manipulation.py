"""
Manipulation module for Space Module Dual-Arm Assembly.
"""

import numpy as np
from typing import List, Dict
from .models import CartesianPose, GraspPlan


class ManipulationController:
    """操作控制器"""
    
    def __init__(self, model, data, kinematics):
        self.model = model
        self.data = data
        self.kinematics = kinematics
    
    def compute_approach_vector(self, object_pos: np.ndarray,
                                approach_height: float = 0.15) -> CartesianPose:
        """计算接近向量"""
        approach_pos = object_pos.copy()
        approach_pos[2] += approach_height
        
        # 默认朝下的姿态
        quat = np.array([0, 1, 0, 0])  # 180度绕x轴旋转
        
        return CartesianPose(position=approach_pos, orientation=quat)
    
    def compute_grasp_pose(self, object_pos: np.ndarray,
                           object_width: float = 0.04) -> GraspPlan:
        """计算抓取位姿"""
        approach_pos = object_pos.copy()
        approach_pos[2] += 0.15
        
        grasp_pos = object_pos.copy()
        
        lift_pos = object_pos.copy()
        lift_pos[2] += 0.1
        
        return GraspPlan(
            approach_pos=approach_pos,
            grasp_pos=grasp_pos,
            lift_pos=lift_pos,
            grasp_width=object_width,
            grasp_force=20.0
        )
    
    def pre_grasp_position(self, object_pos: np.ndarray,
                           safety_margin: float = 0.1) -> np.ndarray:
        """预抓取位置"""
        pre_pos = object_pos.copy()
        pre_pos[2] += safety_margin
        return pre_pos
    
    def gripper_control(self, width: float, steps: int = 50) -> bool:
        """夹爪控制"""
        import mujoco
        
        # 将宽度映射到控制值
        ctrl_value = width * 255 / 0.04  # 0-0.04m映射到0-255
        ctrl_value = np.clip(ctrl_value, 0, 255)
        
        for _ in range(steps):
            self.data.ctrl[7] = ctrl_value
            mujoco.mj_step(self.model, self.data)
        
        return True
    
    def pick_object(self, object_pos: np.ndarray,
                    object_width: float = 0.04) -> bool:
        """拾取物体"""
        # 计算抓取规划
        grasp_plan = self.compute_grasp_pose(object_pos, object_width)
        
        # 移动到接近位置
        self._move_to_cartesian_pos(grasp_plan.approach_pos)
        
        # 打开夹爪
        self.gripper_control(0.04, steps=30)
        
        # 移动到抓取位置
        self._move_to_cartesian_pos(grasp_plan.grasp_pos)
        
        # 闭合夹爪
        self.gripper_control(object_width, steps=30)
        
        # 抬起
        self._move_to_cartesian_pos(grasp_plan.lift_pos)
        
        return True
    
    def place_object(self, place_pos: np.ndarray) -> bool:
        """放置物体"""
        # 移动到放置位置上方
        approach_pos = place_pos.copy()
        approach_pos[2] += 0.1
        self._move_to_cartesian_pos(approach_pos)
        
        # 移动到放置位置
        self._move_to_cartesian_pos(place_pos)
        
        # 打开夹爪
        self.gripper_control(0.04, steps=30)
        
        # 抬起
        retract_pos = place_pos.copy()
        retract_pos[2] += 0.1
        self._move_to_cartesian_pos(retract_pos)
        
        return True
    
    def stack_objects(self, object_positions: List[np.ndarray],
                      stack_height: float = 0.04) -> Dict:
        """堆叠物体"""
        operations = []
        
        for i in range(len(object_positions) - 1):
            # 拾取下一个物体
            self.pick_object(object_positions[i + 1])
            
            # 放置到堆叠位置
            stack_pos = object_positions[0].copy()
            stack_pos[2] += stack_height * (i + 1)
            self.place_object(stack_pos)
            
            operations.append({
                "object_index": i + 1,
                "stack_level": i + 1,
                "position": stack_pos.tolist()
            })
        
        return {
            "total_objects": len(object_positions),
            "operations": operations
        }
    
    def sort_objects(self, objects: List[np.ndarray],
                     target_zones: List[np.ndarray]) -> Dict:
        """分拣物体"""
        operations = []
        
        for i, (obj_pos, zone_pos) in enumerate(zip(objects, target_zones)):
            self.pick_object(obj_pos)
            self.place_object(zone_pos)
            
            operations.append({
                "object_index": i,
                "from": obj_pos.tolist(),
                "to": zone_pos.tolist()
            })
        
        return {
            "sorted_count": len(objects),
            "operations": operations
        }
    
    def _move_to_cartesian(self, target_pose: CartesianPose, steps: int = 100):
        """移动到笛卡尔位姿"""
        import mujoco
        
        # 简化实现：使用逆运动学
        # 实际应该使用IK求解器
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
    
    def _move_to_cartesian_pos(self, target_pos: np.ndarray, steps: int = 100):
        """移动到笛卡尔位置"""
        import mujoco
        
        # 简化实现
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
