"""
Fault recovery module for Space Module Dual-Arm Assembly.
"""

import numpy as np
from typing import Dict


class FaultRecovery:
    """故障恢复控制器"""
    
    def __init__(self, controller):
        self.controller = controller
    
    def fault_recovery(self, fault_type: str, current_state: dict,
                       target_state: dict, max_retries: int = 3) -> Dict:
        """故障恢复"""
        recovery_strategies = {
            "misalignment": self._recover_misalignment,
            "grasp_failure": self._recover_grasp_failure,
            "collision": self._recover_collision,
            "drop": self._recover_drop
        }
        
        if fault_type not in recovery_strategies:
            return {
                "recovered": False,
                "strategy": "unknown",
                "attempts": 0,
                "log": [{"error": f"Unknown fault type: {fault_type}"}]
            }
        
        strategy_func = recovery_strategies[fault_type]
        
        for attempt in range(max_retries):
            result = strategy_func(current_state, target_state)
            result["attempts"] = attempt + 1
            
            if result.get("recovered", False):
                return result
        
        return {
            "recovered": False,
            "strategy": fault_type,
            "attempts": max_retries,
            "log": [{"error": "Max retries exceeded"}]
        }
    
    def _recover_misalignment(self, current: dict, target: dict) -> Dict:
        """恢复对准偏差"""
        log = []
        
        # 计算偏差
        current_pos = np.array(current["position"])
        target_pos = np.array(target["position"])
        error = target_pos - current_pos
        
        log.append({
            "attempt": 1,
            "action": "recalculating_alignment",
            "error": error.tolist()
        })
        
        # 尝试重新对准
        correction = error * 0.5  # 部分修正
        new_pos = current_pos + correction
        
        log.append({
            "attempt": 2,
            "action": "applying_correction",
            "correction": correction.tolist()
        })
        
        # 移动到修正后的位置
        self.controller._move_to_cartesian_pos(new_pos)
        
        return {
            "recovered": True,
            "strategy": "misalignment",
            "log": log
        }
    
    def _recover_grasp_failure(self, current: dict, target: dict) -> Dict:
        """恢复抓取失败"""
        log = []
        
        # 打开夹爪
        self.controller.gripper_control(0.04, steps=30)
        log.append({"attempt": 1, "action": "open_gripper"})
        
        # 重新定位
        self.controller._move_to_cartesian_pos(np.array(target["position"]))
        log.append({"attempt": 2, "action": "reposition"})
        
        # 重新抓取
        self.controller.gripper_control(0.0, steps=30)
        log.append({"attempt": 3, "action": "regrip"})
        
        return {
            "recovered": True,
            "strategy": "regrip",
            "log": log
        }
    
    def _recover_collision(self, current: dict, target: dict) -> Dict:
        """恢复碰撞"""
        log = []
        
        # 后退
        retreat_pos = np.array(current["position"])
        retreat_pos[2] += 0.1  # 向上后退
        self.controller._move_to_cartesian_pos(retreat_pos)
        log.append({"attempt": 1, "action": "retreat"})
        
        # 重新规划路径
        self.controller._move_to_cartesian_pos(np.array(target["position"]))
        log.append({"attempt": 2, "action": "reroute"})
        
        return {
            "recovered": True,
            "strategy": "retreat_reroute",
            "log": log
        }
    
    def _recover_drop(self, current: dict, target: dict) -> Dict:
        """恢复掉落"""
        log = []
        
        # 回到home位置
        self.controller.move_to_home()
        log.append({"attempt": 1, "action": "return_home"})
        
        # 重新定位到目标
        self.controller._move_to_cartesian_pos(np.array(target["position"]))
        log.append({"attempt": 2, "action": "reposition"})
        
        # 重新抓取
        self.controller.gripper_control(0.0, steps=30)
        log.append({"attempt": 3, "action": "regrip"})
        
        return {
            "recovered": True,
            "strategy": "relocate_regrip",
            "log": log
        }
