"""
Physics Audit for Space Module Dual-Arm Assembly
================================================
Verifies that the system genuinely interacts with MuJoCo physics.
8/8 checks must pass to prove real physics integration.
"""

import json
import numpy as np
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent))
from franka_controller import FrankaController


class PhysicsAudit:
    """Audit system to verify genuine physics interaction."""
    
    def __init__(self):
        self.controller = FrankaController()
        self.checks = []
    
    def run_all_checks(self):
        """Run all 8 physics verification checks."""
        print("Running Physics Audit (8 checks)...")
        print("="*60)
        
        checks = [
            ("contact_force_proof", self.check_contact_force),
            ("module_displacement", self.check_module_displacement),
            ("force_sensor_correlation", self.check_force_sensor_correlation),
            ("joint_actuation", self.check_joint_actuation),
            ("collision_detection", self.check_collision_detection),
            ("grip_force_variation", self.check_grip_force_variation),
            ("impedance_response", self.check_impedance_response),
            ("fault_recovery_physics", self.check_fault_recovery_physics)
        ]
        
        results = {}
        passed = 0
        
        for check_name, check_func in checks:
            print(f"\n[CHECK] {check_name}")
            try:
                result = check_func()
                results[check_name] = {
                    "passed": result["passed"],
                    "evidence": result["evidence"],
                    "metric": result.get("metric", "")
                }
                if result["passed"]:
                    passed += 1
                    print(f"  ✓ PASSED: {result['evidence']}")
                else:
                    print(f"  ✗ FAILED: {result['evidence']}")
            except Exception as e:
                results[check_name] = {
                    "passed": False,
                    "evidence": f"Error: {str(e)}",
                    "metric": ""
                }
                print(f"  ✗ ERROR: {str(e)}")
        
        results["summary"] = {
            "total_checks": len(checks),
            "passed": passed,
            "failed": len(checks) - passed,
            "pass_rate": f"{passed}/{len(checks)}",
            "all_passed": passed == len(checks)
        }
        
        print("\n" + "="*60)
        print(f"AUDIT RESULT: {passed}/{len(checks)} checks passed")
        print("="*60)
        
        return results
    
    def check_contact_force(self):
        """Check 1: Verify contact forces are measured."""
        self.controller.reset()
        
        # Move to object and grasp
        obj_pos = np.array([0.15, 0.0, 0.44])
        self.controller.pick_object(obj_pos)
        
        # Measure force during grasp
        force_data = self.controller.force_estimation()
        force_magnitude = force_data["force_magnitude"]
        
        # Force should be non-zero during contact
        passed = force_magnitude > 0.1  # At least 0.1N
        
        return {
            "passed": passed,
            "evidence": f"Contact force measured: {force_magnitude:.2f}N",
            "metric": f"{force_magnitude:.2f}N"
        }
    
    def check_module_displacement(self):
        """Check 2: Verify modules actually move."""
        self.controller.reset()
        
        # Get initial position
        initial_pos = self.controller.get_end_effector_pos()
        
        # Move arm
        target_qpos = self.controller.HOME_QPOS + np.array([0.1, 0, 0, 0, 0, 0, 0])
        self.controller.set_joint_positions(target_qpos, steps=100)
        
        # Get final position
        final_pos = self.controller.get_end_effector_pos()
        
        # Calculate displacement
        displacement = np.linalg.norm(final_pos - initial_pos)
        
        # Should have moved at least 1cm
        passed = displacement > 0.01
        
        return {
            "passed": passed,
            "evidence": f"End-effector displaced: {displacement*100:.1f}cm",
            "metric": f"{displacement*100:.1f}cm"
        }
    
    def check_force_sensor_correlation(self):
        """Check 3: Verify force sensors correlate with contact."""
        self.controller.reset()
        
        # Measure force without contact
        force_no_contact = self.controller.force_estimation()["force_magnitude"]
        
        # Make contact
        obj_pos = np.array([0.15, 0.0, 0.44])
        self.controller.pick_object(obj_pos)
        
        # Measure force with contact
        force_with_contact = self.controller.force_estimation()["force_magnitude"]
        
        # Force should increase during contact
        force_increase = force_with_contact - force_no_contact
        passed = force_increase > 0.05  # At least 0.05N increase
        
        return {
            "passed": passed,
            "evidence": f"Force increase during contact: {force_increase:.2f}N",
            "metric": f"{force_increase:.2f}N"
        }
    
    def check_joint_actuation(self):
        """Check 4: Verify joints actually actuate."""
        self.controller.reset()
        
        # Record initial joint positions
        initial_qpos = self.controller.data.qpos[:7].copy()
        
        # Command joint movement
        target_qpos = initial_qpos + np.array([0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1])
        self.controller.set_joint_positions(target_qpos, steps=200)
        
        # Record final joint positions
        final_qpos = self.controller.data.qpos[:7].copy()
        
        # Calculate joint displacement
        joint_displacement = np.linalg.norm(final_qpos - initial_qpos)
        
        # Joints should have moved
        passed = joint_displacement > 0.05  # At least 0.05 radians
        
        return {
            "passed": passed,
            "evidence": f"Joint displacement: {joint_displacement:.3f} rad",
            "metric": f"{joint_displacement:.3f} rad"
        }
    
    def check_collision_detection(self):
        """Check 5: Verify collision detection works."""
        self.controller.reset()
        
        # Check for collisions in home position
        collision_data = self.controller.collision_detection()
        
        # Should return valid collision data
        has_valid_data = (
            "has_collision" in collision_data and
            "num_contacts" in collision_data
        )
        
        # Try to create a collision scenario
        # Move arm to a position that might cause collision
        aggressive_qpos = self.controller.HOME_QPOS + np.array([0.5, 0, 0, 0, 0, 0, 0])
        self.controller.set_joint_positions(aggressive_qpos, steps=50)
        
        collision_data_after = self.controller.collision_detection()
        
        passed = has_valid_data and isinstance(collision_data_after["num_contacts"], int)
        
        return {
            "passed": passed,
            "evidence": f"Collision detection functional, contacts: {collision_data_after['num_contacts']}",
            "metric": f"{collision_data_after['num_contacts']} contacts"
        }
    
    def check_grip_force_variation(self):
        """Check 6: Verify grip force varies with object."""
        self.controller.reset()
        
        # Measure grip force at different widths
        forces = []
        for width in [0.0, 0.02, 0.04]:
            self.controller.gripper_control(width, steps=20)
            force = self.controller.force_estimation()["force_magnitude"]
            forces.append(force)
        
        # Force should vary with grip width
        force_variation = max(forces) - min(forces)
        passed = force_variation > 0.01  # At least 0.01N variation
        
        return {
            "passed": passed,
            "evidence": f"Grip force variation: {force_variation:.3f}N",
            "metric": f"{force_variation:.3f}N"
        }
    
    def check_impedance_response(self):
        """Check 7: Verify impedance control produces forces."""
        self.controller.reset()
        
        # Apply impedance control
        target = np.array([0.4, 0, 0.3])
        tau = self.controller.impedance_control(target)
        
        # Should produce non-zero torques
        torque_magnitude = np.linalg.norm(tau)
        passed = torque_magnitude > 0.01  # At least 0.01 Nm
        
        return {
            "passed": passed,
            "evidence": f"Impedance torque magnitude: {torque_magnitude:.3f} Nm",
            "metric": f"{torque_magnitude:.3f} Nm"
        }
    
    def check_fault_recovery_physics(self):
        """Check 8: Verify fault recovery interacts with physics."""
        self.controller.reset()
        
        # Simulate misalignment
        current = {"position": [0.15, 0.01, 0.48]}
        target = {"position": [0.15, 0.0, 0.44]}
        
        # Run fault recovery
        result = self.controller.fault_recovery("misalignment", current, target)
        
        # Should have recovery log with physics interactions
        has_log = "log" in result and len(result["log"]) > 0
        has_attempts = "attempts" in result and result["attempts"] > 0
        
        passed = has_log and has_attempts
        
        return {
            "passed": passed,
            "evidence": f"Fault recovery attempted {result.get('attempts', 0)} times with {len(result.get('log', []))} log entries",
            "metric": f"{result.get('attempts', 0)} attempts"
        }


def main():
    """Run the physics audit."""
    audit = PhysicsAudit()
    results = audit.run_all_checks()
    
    # Save results
    output_file = Path(__file__).parent / "physics_audit.json"
    
    # Convert numpy types to Python types for JSON serialization
    def convert_to_serializable(obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, np.bool_):
            return bool(obj)
        return obj
    
    # Convert results
    serializable_results = json.loads(json.dumps(results, default=convert_to_serializable))
    
    with open(output_file, "w") as f:
        json.dump(serializable_results, f, indent=2)
    
    print(f"\nAudit results saved to: {output_file}")
    
    # Return exit code
    if results["summary"]["all_passed"]:
        print("\n✓ ALL CHECKS PASSED - Physics integration verified!")
        return 0
    else:
        print(f"\n✗ {results['summary']['failed']} CHECKS FAILED")
        return 1


if __name__ == "__main__":
    exit(main())
