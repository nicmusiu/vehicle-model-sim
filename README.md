# 🚗 Simplified Vehicle Model

This repository provides a framework to simulate simplified vehicle dynamics using **Open Loop Simulation**.
Choose model inputs, simulate and verify results.

---

<!-- ## 📦 Features

- Support for multiple simplified vehicle models (e.g., tricycle model)
- Optional **Limited Slip Differential** modeling
- Combined slip tire force calculations
- Easily configurable simulation and plotting settings via JSON

--- -->

## 🚀 Getting Started

### 1. Configure the Simulation

Edit the `sim_setting.json` file to define:

- `integration timestep` — numerical integration step
- `initial state` — initial conditions for the vehicle model
- `model definition`:
  - Enable/disable **Limited Slip Differential** (`LSD:true` for tricycle model)
  - Enable/disable **combined slip** calculation
- `plotting options` — select which variables to visualize

---

### 2. Adjust Model Parameters (Optional)

For deeper customization, you can modify the model’s physical parameters
in the folder **params**:

- Define the **engine torque curve**
- Tune **vehicle parameters** (mass, inertia, tire characteristics, etc.)

---

### 3. Define model Input

In the script (Section **%%INPUT**):
```matlab
main.m
```
select the input you want to provide for the model:
- steering angle (u.steering)
- Throttle (u.D)
- Brake (u.B)
- Gear (u.gear)

### 3. Run the Simulation

Execute the main script in MATLAB:

```matlab
main.m
```

## ✅ In progress

New features will be available soon:

- Standard and Enhanced **Kinematic Models**
- Linear Model in **Matrix Form**
- Support for **Different Integrators** (e.g., Euler, Runge-Kutta 2, ...)
- Tools for **Stability Analysis**
