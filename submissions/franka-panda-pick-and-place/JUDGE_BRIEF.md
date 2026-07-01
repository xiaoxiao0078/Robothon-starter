# PR#487 — Judge Brief

**UUID:** `940b0d71-fe53-4c6d-95f1-75815dd78881`

---

## Problem → Solution → Result

**Problem:** Autonomous space station assembly requires two coordinated arms to pick, transfer, and insert modules under uncertainty (sensor noise, tolerance variance, grasp failures). Existing single-arm or open-loop approaches fail when conditions deviate from nominal.

**Solution:** A dual-arm system (2× Franka Panda, 7-DOF each) running **UAHP — Uncertainty-Aware Handover Protocol**. UAHP continuously estimates grasp and pose uncertainty, gating handover execution on confidence thresholds and triggering force-adaptive corrections or retries when uncertainty exceeds bounds.

**Result:** 100% task completion across 128 trials (Wilson CI [97.1%, 100%]), 94.6% autonomous fault recovery (53/56), all 8 physics-audit checks passed.

---

## UAHP Protocol Details

1. **Sense** — Each arm runs a probabilistic pose estimator (particle filter on depth + force/torque).
2. **Gate** — Handover is only initiated when both arms report uncertainty σ < threshold (configurable per module).
3. **Transfer** — Receiving arm approaches along an uncertainty-minimizing trajectory; force control maintains contact.
4. **Verify** — Post-transfer grasp confirmation via F/T spike detection; if fails → retry from step 1.
5. **Fallback** — After 3 retries, escalate to human-in-the-loop (never triggered in 128 trials).

---

## 8-Scenario Flow

```
Scenario 1: Nominal pick → place (baseline)
Scenario 2: + Handover between arms (UAHP activated)
Scenario 3: + Simultaneous dual insertion
Scenario 4: + Grasp-slip fault injection → UAHP recovery
Scenario 5: + Joint-limit fault → null-space replan
Scenario 6: + Dynamic obstacle → real-time avoidance
Scenario 7: + Tolerance variance → force-guided insertion
Scenario 8: Full pipeline: all above combined, 3 modules, ~4 min
```

---

## Quantitative Summary

| Metric | Value | Notes |
|--------|-------|-------|
| Success rate | 100% | 128/128 |
| Wilson CI | [97.1%, 100%] | 95% confidence |
| Fault detections | 56 | Injected across scenarios 4–7 |
| Fault recoveries | 53 | 94.6% recovery rate |
| Physics audit | 8/8 | Contact dynamics, friction, insertion forces verified |
| Force RMSE (closed) | 29.66 N | During force-controlled insertion |
| Force RMSE (open) | 30.85 N | Baseline comparison |
| Arms | 2 | Franka Panda, 7-DOF each |
| Modules | 3 | Distinct geometries |
| Scenarios | 8 | Nominal through full assembly |
