# Cascade Temperature Control with Smart Air Flow — IMT Nord Europe (DNM–DMI 2025–2026)

Robust, physically-motivated simulation and embedded control for a small **8 L hotbox** that heats and dries a coated metal plate while respecting an **air-temperature cap**.  
The project includes:

- A **low-order thermal model** (air, wall, 1-D product conduction + radiation)
- A **cascade controller** (outer PI for core → air set-point, inner PID for air → heater) with rate limits, derivative filtering, and anti-windup
- A **supervisory airflow policy** (valves + fan) regulated by a ∆p PI
- A **MicroPython reference implementation** for a **Raspberry Pi Pico W** that maps a temperature to a hobby **servo** with smoothing, hysteresis, deadband, and slew limiting
- A **LaTeX report** with figures, cross-references, and “back to text” anchors

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Repo Structure](#repo-structure)
- [Hardware](#hardware)
- [Wiring (Pico W + Servo)](#wiring-pico-w--servo)
- [Firmware (MicroPython) — Getting Started](#firmware-micropython--getting-started)
- [Simulation & Plots](#simulation--plots)
- [LaTeX Report — Build](#latex-report--build)
- [Control Design (Brief)](#control-design-brief)
- [Tuning Cheatsheet](#tuning-cheatsheet)
- [Safety Notes](#safety-notes)
- [Troubleshooting](#troubleshooting)
- [License](#license)
- [Cite](#cite)

---

## Overview

We aim to **reach and hold a core temperature** (e.g., 65 °C) while keeping **box air ≤ 70 °C** to protect the coating and enclosure.  
The controller is **cascade**: the outer loop shifts the **air set-point** from core error; the inner **PID** commands heater power. A supervisory airflow layer manages purge and safety behavior via fan/valves.

**Physics.** Well-mixed air, lumped wall, 1-D conduction across half-thickness, convection via  
\( h = \tfrac{k}{L}\mathrm{Nu}(Re,Pr) \), and surface↔wall radiation \( \propto T^4 \).

---

## Features

- **Numerically robust control**: derivative filter + cap, PI/PID anti-windup, slew limits (heater & set-points)
- **Airflow supervisor**: heat-up / track / safety modes; ∆p PI drives realistic mass flow  
  \( \dot m=\rho\,C\!dA\sqrt{2\Delta p/\rho} \)
- **Back-to-text figure links** in the PDF (quick navigation for reviewers)
- **Pico W servo demo**: hybrid thermostat (EMA smoothing, hysteresis, deadband, ramp) — no Wi-Fi needed

---

## Repo Structure

> Adjust folder names to match your actual layout. In the LaTeX file, keep image paths consistent (e.g., `\includegraphics{figs/fig_product_temps.png}`).

---

## Hardware

- **Raspberry Pi Pico W** (MicroPython)
- **Hobby servo** (5 V)
- **5 V supply** for the servo (**share GND** with Pico)
- (Optional) External temperature sensor (for production: NTC on ADC0–3 or a digital sensor, e.g., I²C)
- Prototype hotbox with **two inlets + two outlets**, fan, heater, pressure sensor (for full system)

---

## Wiring (Pico W + Servo)

- Servo **+5 V** → 5 V supply  
- Servo **GND** → supply GND **and** Pico GND (common ground)  
- Servo **signal** → Pico **GP1**  
- Internal temperature sensor is read via **ADC4** (for demo). Replace with an external sensor for real control.

> The demo firmware **does not use Wi-Fi**. It runs an on-board thermostat mapping temperature to servo position.

---

## Firmware (MicroPython) — Getting Started

1. **Flash MicroPython** to Pico W (UF2 from the official site).
2. Copy the script to the board (e.g., `code/pico_thermostat_servo.py`) using **Thonny**, **mpremote**, or **rshell**.
3. Power the servo from 5 V and share ground with the Pico.
4. Run the script. You’ll see telemetry like:

### Key Settings (inside the script)

| Parameter            | Default | Purpose                          |
|---------------------|:------:|----------------------------------|
| `SERVO_PIN`         | `1`    | Servo PWM (GP1)                  |
| `SERVO_FREQ_HZ`     | `50`   | 50 Hz hobby standard             |
| `CLOSED_US`/`OPEN_US` | `1000/2000` | Endpoints (µs)             |
| `REVERSED`          | `False`| Flip direction                   |
| `T_LOW` / `T_HIGH`  | `18/22`| Linear mapping zone (°C)         |
| `TEMP_ALPHA`        | `0.25` | EMA smoothing                    |
| `EDGE_HYST`         | `0.5`  | Hysteresis around edges (°C)     |
| `DEADBAND_US`       | `6`    | Ignore tiny pulse changes        |
| `RAMP_US_PER_S`     | `400`  | Slew limit (µs/s)                |
| `WARMUP_S`          | `2.0`  | ADC settle time                  |

**Robustness summary:** EMA smoothing + hysteresis reduce chatter, deadband avoids tiny writes, and slew limiting protects the mechanics.

---

## Simulation & Plots

- Export the following **PNG** files into `figs/`:
- `fig_product_temps.png` (core vs. surface vs. target)
- `fig_air_control.png` (air, air set-point, cap)
- `fig_heater_fan.png` (heater power + fan duty)
- `fig_inlet_flows.png`, `fig_outlet_flows.png`, `fig_total_flow_dp.png`
- `fig_heatmap.png` (1-D through-thickness heat map)

> Ensure the names match the LaTeX `\includegraphics{...}` calls.

---

## LaTeX Report — Build

We recommend `latexmk`:

```bash
cd report
latexmk -pdf -interaction=nonstopmode main.tex
# or:
pdflatex main && bibtex main && pdflatex main && pdflatex main

