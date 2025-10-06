# Cascade Temperature Control with Smart Air Flow  

## IMT Nord Europe — DNM–DMI 2025–2026

A compact **thermal control project** combining simulation, control design, and embedded implementation.  
The goal: heat and dry a coated metal plate in a small **8 L hotbox** while keeping air ≤ 70 °C for safety.

---

## 🔍 Overview

- **Thermal model:** air, wall, and product (1-D conduction + radiation)  
- **Control:** cascade PI/PID loops  
  - Outer PI: core → air set-point  
  - Inner PID: air → heater power  
- **Airflow system:** fan + valves managed by a smart supervisor (heat-up / hold / safety)  
- **Embedded version:** MicroPython control on **Raspberry Pi Pico W** driving a hobby servo

---

## 🧠 Key Features

- Safe and stable heating control with power/temperature caps  
- Anti-windup, rate-limiting, and derivative filtering  
- Realistic airflow simulation (Δp PI and orifice flow)  
- Ready-to-run **MicroPython demo** — no Wi-Fi required  
- Fully documented **LaTeX report** with cross-references and figures

---

## ⚙️ Hardware Setup

| Component | Role |
|------------|------|
| Raspberry Pi Pico W | Controller board |
| Hobby servo (5 V) | Air valve actuator |
| Temperature sensor | Internal ADC or NTC |
| Heater + Fan | Thermal actuation system |

> Servo signal → **GP1**, powered from 5 V (share GND with Pico)

---

## 💻 Quick Start

1. Flash **MicroPython** on the Pico W  
2. Upload `pico_thermostat_servo.py`  
3. Power the servo (5 V + GND) and run the script  
4. Monitor live telemetry like:
