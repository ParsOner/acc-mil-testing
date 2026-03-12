# 🚗 ACC Testing & Validation — MIL Framework

> **Testing and Validation of Automated Road Vehicles** · Final Project 2025–2026
>
> 
## What is this?

A **Model-in-the-Loop (MIL)** test framework for an **Adaptive Cruise Control (ACC)** system. The controller is evaluated across **210 simulation scenarios** generated via Latin Hypercube Sampling (LHS), extracting safety and comfort KPIs from each run.

```
Bus.m  ──►  CCToACCFinal1.slx  ──►  RUN_ACC_LHS.m  ──►  resultsLHS
(bus types)    (Simulink model)      (LHS + batch sim)    (KPI table)
```

---

## 📁 Files

| File | Description |
|---|---|
| `CCToACCFinal1.slx` | Simulink model – ACC controller, vehicle dynamics, radar, KPI block |
| `Bus.m` | Bus object definitions – **must run before the model** |
| `RUN_ACC_LHS.m` | Batch runner: samples 7 parameters, simulates 210 scenarios, returns KPIs |

---

## ⚙️ Parameter Space (LHS – 7 dimensions)

| Parameter | Range | Unit |
|---|---|---|
| Speed setpoint | 15 – 35 | m/s |
| Time gap | 0.9 – 2.3 | s |
| Default spacing | 2.5 – 7.0 | m |
| Vehicle mass | 900 – 1600 | kg |
| Aero drag `Cd·Af` | 0.45 – 0.805 | m² |
| Rolling resistance `Cr` | 0.007 – 0.016 | – |
| Road slope `θ` | −0.035 – +0.07 | rad |

---

## 📊 KPIs Extracted

| KPI | Meaning |
|---|---|
| `TTC_min` | Minimum Time-to-Collision — core safety indicator |
| `viol_rate` | Fraction of time safe distance was violated |
| `a_max` | Peak acceleration during the run |
| `a_min` | Peak braking (most negative value) |

---

## 🔑 Key Findings

- **Time gap is the #1 safety driver** — gaps < 1.5 s frequently produce unsafe TTC (r = 0.81, p = 1.3e-43)
- `TTC_min` and `viol_rate` are nearly perfectly anti-correlated (r = −0.97)
- Hard braking is a *consequence* of low TTC, not the cause
- Vehicle speed has negligible effect on TTC_min (r = 0.07, p = 0.32)
- LHS gives uniform parameter coverage — no clustering, no wasted runs

---

## 🚀 How to Run

```matlab
% 1 — Load bus definitions (required first)
Bus

% 2 — Open the model
open_system('CCToACCFinal1')

% 3 — Run 210 LHS simulations
resultsLHS = RUN_ACC_LHS(210);

% 4 — Explore results
T = struct2table(resultsLHS);
scatter(T.time_gap, T.TTC_min, 20, T.viol_rate, 'filled')
```

**Requirements:** MATLAB + Simulink + Automated Driving Toolbox + Statistics and Machine Learning Toolbox
