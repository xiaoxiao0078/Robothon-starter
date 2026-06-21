# Franka Panda Smart Manipulation

> 🤖 基于Franka Emika Panda机械臂的智能抓取放置系统

[![Python 3.12](https://img.shields.io/badge/python-3.12-blue.svg)](https://www.python.org/downloads/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-simulation-green.svg)](https://mujoco.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Tests: 77/77 passing](https://img.shields.io/badge/tests-77%2F77%20passing-brightgreen.svg)](tests)

## 📋 Overview

A comprehensive robotic manipulation system built on the Franka Emika Panda platform in MuJoCo. This project implements 25 core tasks covering:

- **Kinematics**: Forward kinematics, Jacobian computation
- **Motion Planning**: Joint/Cartesian interpolation, minimum-jerk trajectories
- **Obstacle Avoidance**: Artificial potential field method
- **Grasp Planning**: Approach vectors, grasp poses, pre-grasp positions
- **Manipulation**: Pick-and-place, stacking, sorting
- **Advanced Control**: Impedance control, visual servoing, force estimation
- **Skill Learning**: Learning from demonstration via DMP approximation
- **Task Orchestration**: Complete pipeline execution

## 🎯 Task Completion (25/25)

| # | Task | Status | Description |
|---|------|--------|-------------|
| 1 | Model Loading | ✅ | Load MuJoCo Franka Panda model |
| 2 | Joint Info | ✅ | Query joint names, limits, DOF |
| 3 | Forward Kinematics | ✅ | EE pose from joint angles |
| 4 | Jacobian | ✅ | Linear + angular velocity Jacobian |
| 5 | Joint Control | ✅ | Position control with simulation |
| 6 | Joint Interpolation | ✅ | Linear interpolation in joint space |
| 7 | Cartesian Interpolation | ✅ | Linear + SLERP interpolation |
| 8 | Min-Jerk Trajectory | ✅ | 5th-order polynomial trajectory |
| 9 | Obstacle Avoidance | ✅ | Artificial potential field |
| 10 | Workspace Analysis | ✅ | Monte Carlo workspace sampling |
| 11 | Approach Vector | ✅ | Top-down approach computation |
| 12 | Grasp Pose | ✅ | Full grasp plan (approach-grasp-lift) |
| 13 | Pre-Grasp Position | ✅ | Safe pre-grasp with margin |
| 14 | Gripper Control | ✅ | Open/close with width mapping |
| 15 | Force Estimation | ✅ | Contact force from sensors |
| 16 | Pick Object | ✅ | Complete pick pipeline |
| 17 | Place Object | ✅ | Complete place pipeline |
| 18 | Stack Objects | ✅ | Multi-object stacking |
| 19 | Sort Objects | ✅ | Zone-based sorting |
| 20 | Trajectory Recording | ✅ | Motion data with force logging |
| 21 | Collision Detection | ✅ | Environmental collision check |
| 22 | Impedance Control | ✅ | Cartesian impedance with K/D |
| 23 | Visual Servoing | ✅ | IBVS with affine approximation |
| 24 | Skill Learning | ✅ | DMP from demonstrations |
| 25 | Task Orchestration | ✅ | Complete pipeline execution |

## 🏗️ Architecture

```
franka_controller.py          # Core controller (25 tasks)
test_franka_controller.py     # 77 unit tests
record_demo.py                # Video recording script
```

## 🚀 Quick Start

```bash
pip install mujoco numpy imageio pytest

# Run tests
python -m pytest test_franka_controller.py -v

# Record demo
python record_demo.py
```

## 🔧 Technical Details

### Robot Model
- **Platform**: Franka Emika Panda (7-DOF + 2-finger gripper)
- **Model Source**: MuJoCo Menagerie (BSD-3-Clause)
- **Joint Limits**: ±2.8973 rad (joints 1,3,5,7), ±1.7628 rad (joint 2)
- **Gripper**: Parallel-jaw with tendon coupling

### Key Algorithms
1. **Minimum-Jerk Trajectory**: 5th-order polynomial for smooth motion
2. **Artificial Potential Field**: Attractive + repulsive forces for obstacle avoidance
3. **Impedance Control**: Stiffness/damping-based force regulation
4. **IBVS**: Image-based visual servoing with affine Jacobian approximation
5. **DMP**: Dynamic Movement Primitives for skill learning from demonstration

### Test Coverage
- 77 unit tests across 25 task categories
- Performance benchmarks (<10ms FK, <20ms Jacobian)
- All tests passing ✅

## 📊 Benchmark Results

| Metric | Value |
|--------|-------|
| Forward Kinematics | <10ms per call |
| Jacobian Computation | <20ms per call |
| Trajectory Planning | <20ms (100 points) |
| Workspace Volume | ~0.3 m³ |
| Max Reach | ~0.855m |

## 📹 Demo Video

The demo showcases:
1. Robot initialization and workspace visualization
2. Three pick-and-place operations with different objects
3. Object stacking capability
4. Smooth minimum-jerk trajectories
5. Gripper open/close operations

## 👥 Team

- **Xiaoxiao Team** - Robotics & AI

## 📄 License

MIT License - See [LICENSE](LICENSE) for details.
