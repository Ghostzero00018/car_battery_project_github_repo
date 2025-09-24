# Project Overview

This project develops and validates a thermal-control strategy for a battery module placed in a **hot box**. We follow two phases:

1) **Simulation (MATLAB/Simulink)**  
   We model the battery enclosure heated to a target temperature and then regulated at setpoint to mimic real car-battery production. The model includes:
   - **Energy balance** for internal air and battery surface with convective losses to ambient:
     \[
     C_\text{th}\,\frac{dT}{dt} = P_\text{heater} + P_\text{load} - hA\,(T - T_e) - \dot{m} c_p (T - T_e)
     \]
   - **Forced convection** via a 12 V axial fan (Sunon family): fan curve + enclosure flow resistance to estimate **Q** and infer **h**.
   - **Leak model** using **single-hole + unavoidable leaks** (orifice equation) to compare with a side-vent design.
   - **Disturbances**: ambient changes, door open, cold air ingress.
   - **Control**: PI/PID (anti-windup, bumpless transfer, output clamping), soft-start heater, and fan ramp to minimize overshoot.

   **Targets**  
   - Reach setpoint quickly with **< 1% overshoot**.  
   - Overshoot (if any) settles within **10–20 min**.  
   - Stable steady-state with specified ΔT across the volume.

2) **Prototype (hardware-in-the-loop)**
   We build a compact bench to validate the model and tune the controller.

   ## Hardware

   - **MCU:** Raspberry Pi **Pico W** (Wi-Fi optional for logging/UI).
   - **Heater:** compact **resistive element** (12–24 V) sized from the sim (power margin ≥ 30%).
   - **Fan:** 12 V **Sunon** axial (e.g., 60×60×38 mm ~96 m³/h). Fan is PWM/voltage controlled via a logic-level **MOSFET** or driver.
   - **Sensors:**
     - **Temperature**: NTC or digital (e.g., DS18B20) near battery surface + air in/out.
     - **Differential pressure**: to estimate airflow and verify fan curve vs. box resistance.
   - **Power:** 12 V supply for fan/heater, 5 V/USB for Pico W; common ground.
   - **Safety:** heater fuse, thermal cutoff, over-temp software interlock, fan-failure detect.

   ### Control Functions (on Pico W)

   - Read **temperature** and **Δp** (airflow proxy).
   - Drive **fan PWM** for airflow control.
   - Modulate **heater power** (PWM) for temperature control.
   - **State machine**: IDLE → PREHEAT (open-loop) → HOLD (closed-loop) → FAULT.
   - **Anti-windup** for the heater loop; **rate limiting** to avoid thermal shock.
   - **Data logging**: time, T_in/T_out/T_batt, PWM_fan, PWM_heater, Δp, setpoint, error.

---

## Simulation Details (MATLAB/Simulink)

- **Inputs:** ambient \(T_e\), heater power limit \(P_\text{max}\), fan curve \(Q(\Delta p)\), box leakage area, duct/orifice coefficients, \(C_\text{th}\), \(h(Q)\) correlations.
- **Outputs:** warm-up time, setpoint tracking, overshoot, steady-state power, airflow, pressure.
- **Assumptions:** air as ideal gas; lumped thermal mass for early design; later refine with multi-node model if gradient across battery is non-negligible.
- **Correlations:** internal forced convection (Nu–Re–Pr) to link \(Q\) to \(h\); orifice flow for the leak path; fan + system curve intersection for operating point.

> We compare **single intake + leak** vs **side vents** to see effects on warm-up, Δp, and final power draw.

---

## Control Strategy

- **Heater loop (primary):** PI/PID on air or battery-surface temperature with:
  - **Soft-start** to reduce initial surge.
  - **Anti-windup** (integrator clamping or back-calculation).
  - **Output limits** (0…\(P_\text{max}\)) and **slew rate**.
- **Fan loop (secondary):**
  - Option A: fixed airflow (constant PWM) chosen from sizing study.
  - Option B: supervisory control to adjust airflow if Δp or T-gradient exceeds bounds.
- **Fault handling:** sensor timeouts, implausible readings, over-temp, fan stall → transition to FAULT (heater off, fan purge).

---

## Validation Plan

1. **Oven/hot-box step test:** measure time constant, overshoot, steady-state power; identify parameters \(C_\text{th}\), \(h\).
2. **Airflow characterization:** map PWM → Δp → Q; compare to datasheet/system curve.
3. **Leak sensitivity:** vary hole diameter (taped inserts) and log effects on warm-up and steady state.
4. **Setpoint campaign:** multiple setpoints (e.g., 40 °C, 50 °C, 60 °C) and ambient shifts (±5 °C).
5. **Acceptance criteria:** < 1% overshoot, settle within 10–20 min, steady-state error < ΔT_target, safety interlocks proven.

**Logged metrics:** \(T(t)\), Δp(t), PWM_fan, PWM_heater, \(P_\text{elec}\), events (faults, state changes). Export CSV for analysis.

---

## Repository Structure (proposed)
