# Space Module Dual-Arm Assembly

## Project Overview

This project demonstrates a **dual-arm robotic system** using two Franka Emika Panda arms to collaboratively assemble space station modules. The system showcases coordinated manipulation, handoff operations, and precise assembly tasks.

## Key Features

- **Dual-Arm Coordination**: Two 7-DOF robotic arms working in sync
- **Space Module Assembly**: Blue, red, and green modules represent space station components
- **Multi-Phase Task**: 8-step assembly sequence including reach, grasp, lift, handoff, and assembly
- **Force Feedback**: Real-time gripper force monitoring
- **Visual Feedback**: Status panels showing task phase, gripper state, and progress

## Task Sequence

1. **Home Position** - Arms return to initial configuration
2. **Reach Blue Module** - Left arm approaches blue space module
3. **Grasp Module** - Left arm grasps the blue module
4. **Lift & Reach** - Left arm lifts module, right arm approaches red module
5. **Handoff** - Module transfer between arms
6. **Assembly** - Precise module alignment and connection
7. **Grab Green** - Right arm grasps green module
8. **Final Assembly** - Complete space module assembly

## Technical Implementation

- **IK Solver**: Analytical Jacobian-based inverse kinematics for both arms
- **Collision Avoidance**: Joint limit enforcement and workspace constraints
- **Trajectory Planning**: Smooth cosine interpolation for natural motion
- **Rendering**: MuJoCo physics engine with custom camera angles

## Files

- `franka_controller.py` - Main robot control logic
- `test_franka_controller.py` - Unit tests
- `demo.mp4` - Demo video showing full assembly sequence
- `README.md` - This file
- `JUDGE_BRIEF.md` - Technical details for judges

## Robot Configuration

- **Left Arm**: Base at x=-0.5, reaching to workspace center
- **Right Arm**: Base at x=+0.5, reaching to workspace center
- **Modules**: Positioned at z=0.7, centered between arms
- **Grippers**: Parallel jaw grippers with force control

## Dependencies

- MuJoCo physics engine
- NumPy for matrix operations
- Matplotlib for visualization overlay
- PIL for image processing

## Author

xiaoxiao0078 - Robothon 2026 Competition Entry
