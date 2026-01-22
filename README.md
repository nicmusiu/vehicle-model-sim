# 🚗 Simplified Vehicle Dynamics Models

This repository provides a modular framework to simulate **simplified vehicle dynamics models** in **open-loop** configuration.

You can select model inputs, run simulations, and analyze the results to:
- validate experimental data,
- build simulation environments (e.g. gyms for black-box model training),
- prototype or benchmark **model-based controllers and estimators**,
- compare different modeling assumptions and levels of complexity.

---

## 📁 Repository Structure

The repository is organized into the following main folders:

1. **`compare_simplified_models`**  
   Several simplified vehicle models are simulated and compared, including:
   - kinematic single-track model,
   - linear and nonlinear single-track models,
   - tricycle model with a locked differential.

   ⚠️ A **single set of parameters** is shared among all models. As a result, the models do **not** represent the same physical vehicle exactly, as the exact tire parameters used within the work are protected.

2. **`run_stability_analysis`**  
   Tools to analyze **system poles and stability**.  
   Use this folder to evaluate whether the chosen parameters and integration timestep yield stable lateral dynamics.  
   The results produced in `compare_simplified_models` can be reused here.

3. **`run_sim_using_complete_model`**  
   A more detailed vehicle model, including:
   - realistic driver inputs (steering, pedals, gear selection),
   - Limited Slip Differential (LSD),
   - combined slip tire effects,
   - engine torque and RPM dynamics,
   - 🚧 future improvements: longitudinal slip dynamics will be included in the framework!

---

## 🚀 Getting Started


### (1) Model Parameter Customization (Optional)

For deeper customization, model parameters can be modified in the **`params`** folder:

- Define the **engine torque curve**
- Tune **vehicle parameters** (mass, inertia, tire characteristics, etc.)
- Select among different **numerical integrators**, available in the `utils` folder:
  - Euler
  - RK2
  - RK3
  - RK4

---

### (2) How to Run a Simulation

1. Edit the `sim_setting.json` file to define the simulation settings, such as:
   - integration timestep,
   - initial state,
   - simulation horizon and options.
2. Run `main.m`.
3. Analyze the generated plots and logged results.

---
