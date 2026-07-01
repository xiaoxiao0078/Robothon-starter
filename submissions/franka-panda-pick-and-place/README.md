# Space Module Dual-Arm Assembly

**UUID: 940b0d71-fe53-4c6d-95f1-75815dd78881**

---

## 🎯 Project Overview

A dual-arm robotic system for space module assembly using the **Uncertainty-Aware Handover Protocol (UAHP)**. The system uses **two Franka Panda arms** to collaboratively assemble **three space modules** with closed-loop force feedback and **92.3% fault recovery**.

### 🤖 Robot Platform

| Component | Specification |
|-----------|---------------|
| **Robot Arms** | 2 × Franka Panda (7-DOF each) |
| **Modules** | 3 space modules (Blue, Green, Red) |
| **Control** | Closed-loop force control with UAHP |
| **Simulation** | MuJoCo 3.x |
| **Actuators** | Position-controlled (kp=4500/3500/2000) |

### 📊 Key Results

| Metric | Value |
|--------|-------|
| **Success Rate** | **100%** (128/128) |
| **Wilson CI 95%** | [97.1%, 100%] |
| **Force RMSE** | 29.66N (closed-loop) |
| **Fault Recovery** | **92.3%** (48/52) |
| **Ablation Improvement** | +4% force precision |

---

## 🏆 Why This Matters

### The Problem

Space module assembly is challenging because:
- **Dual-arm coordination**: Two arms must work together precisely
- **Uncertainty handling**: Grasp stability, velocity, and alignment vary
- **Fault tolerance**: System must recover from failures gracefully

### Our Solution

We built a **UAHP-driven dual-arm assembly system** that:
1. **Estimates uncertainty** in real-time for grasp stability, velocity, and alignment
2. **Selects adaptive strategies** (fast transfer, slow align, pause replan, emergency stop)
3. **Recovers from faults** with automatic re-grasping (92.3% success)
4. **Achieves 100% success** in 128-trial benchmark

---

## 🔬 Technical Approach

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    UAHP Control Pipeline                     │
├─────────────────────────────────────────────────────────────┤
│  Uncertainty Estimation Layer                                │
│  ├── Grasp stability estimation                             │
│  ├── Velocity estimation                                    │
│  └── Alignment estimation                                   │
├─────────────────────────────────────────────────────────────┤
│  Decision Layer                                              │
│  ├── HCS (Handover Confidence Score) computation            │
│  ├── Strategy selection (fast/slow/pause/emergency)         │
│  └── Adaptive parameter adjustment                          │
├─────────────────────────────────────────────────────────────┤
│  Control Layer                                               │
│  ├── IK solver (Jacobian pseudo-inverse)                    │
│  ├── Trajectory planning (minimum-jerk)                     │
│  ├── Force control (touch sensor feedback)                  │
│  └── Fault recovery (automatic re-grasping)                 │
└─────────────────────────────────────────────────────────────┘
```

### Key Innovations

1. **UAHP (Uncertainty-Aware Handover Protocol)**
   - Real-time uncertainty estimation
   - Adaptive strategy selection
   - 92.3% fault recovery rate

2. **Closed-Loop Force Control**
   - Touch sensor feedback (4 sensors)
   - Fault detection (force < 0.5N)
   - Automatic re-grasping on failure

3. **Dual-Arm Coordination**
   - Coordinated trajectory planning
   - Collision avoidance
   - Safe module transfer between arms

---

## 📈 Results at a Glance

### 128-Trial Benchmark

| Metric | Closed-Loop | Open-Loop |
|--------|-------------|-----------|
| **Success Rate** | 100% | 100% |
| **Wilson CI** | [97.1%, 100%] | [97.1%, 100%] |
| **Force RMSE** | 29.66N | 30.85N |
| **Faults Detected** | 52 | 0 |
| **Faults Recovered** | 48 | 0 |

### Ablation Study

| Configuration | Success Rate | Force RMSE | Recovery Rate |
|---------------|--------------|------------|---------------|
| **Full UAHP** | 100% | 29.66N | 92.3% |
| **No Recovery** | 100% | 30.85N | 0% |
| **No Force Feedback** | 100% | 35.12N | 0% |
| **Open-Loop** | 100% | 30.85N | 0% |

---

## 🚀 How to Run

### Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Run demo
python franka_controller.py

# Run benchmark
python benchmark.py
```

### MuJoCo Visualization

```bash
# Launch MuJoCo viewer
python -m mujoco.viewer --model scene_dual_v5.xml
```

---

## 📁 Files

| File | Description |
|------|-------------|
| `README.md` | Project overview |
| `JUDGE_BRIEF.md` | Judge evaluation summary |
| `EVALUATION_GUIDE.md` | Detailed evaluation guide |
| `franka_controller.py` | Main controller (1500+ lines) |
| `benchmark_results.json` | 128-trial benchmark data |
| `physics_audit.json` | Physics verification results |
| `test_results.json` | Test suite results |
| `scene_dual_v5.xml` | MuJoCo scene |
| `demo.mp4` | Demo video (20s, 1080p) |
| `test_franka_controller.py` | Test suite (77 tests) |

---

## 📊 Competition Entry

| Field | Value |
|-------|-------|
| **UUID** | 940b0d71-fe53-4c6d-95f1-75815dd78881 |
| **Project Name** | Space Module Dual-Arm Assembly |
| **Team** | xiaoxiao0078 |
| **Submission Date** | 2026-06-25 |
| **Version** | v34 |

---

## 🎥 Demo Video

**Duration**: 20 seconds  
**Resolution**: 1920×1072  
**Frame Rate**: 30 fps  

The demo showcases:
1. **Dual-arm coordination** for space module assembly
2. **UAHP protocol** with real-time uncertainty estimation
3. **Fault recovery** in real-time (92.3% success)
4. **HUD overlay** with force readings and progress

---

**UUID: 940b0d71-fe53-4c6d-95f1-75815dd78881**
