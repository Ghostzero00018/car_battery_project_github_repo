# notes.md — Working Notes & Operating Tips

Version: 0.1 (draft)

These notes capture practical tips, gotchas, and short procedures for the **hot-box battery thermal control** prototype.

---

## 1) Safety First (must-read)

- **Heater fuse & thermal cutoff**: verify correct rating before any test.
- **Never** run the heater unattended on first bring-up. Keep a temp probe + fire extinguisher nearby.
- Ensure **fan spins** before enabling high heater duty. If fan fails, controller should **FAULT** (heater off).
- Keep sensor wiring & tubing away from **hot surfaces** and **12 V power** runs.

---

## 2) Power-On Checklist (short)

1. Visual check: polarity, MOSFET orientation, TVS direction, fuse values.
2. Power rails: 12 V, then 5 V (if used), confirm Pico **3.3 V**.
3. Flash firmware that starts **PWM = 0** and checks sensors.
4. Verify sensors:
   - DS18B20: detect all addresses, read sane values (20–35 °C at room).
   - dP sensor (I²C): show zero-ish Δp with open ports.
5. Fan test: 20% → 50% PWM; confirm airflow & current draw.
6. Heater test: start at 10–20% duty, watch temperature rise & MOSFET case temp.

---

## 3) Quick Calibration Hints

- **DS18B20**: place all sensors together at room temp → record offsets; repeat in warm environment (~40–50 °C).
- **Differential pressure**: with both ports vented together, record zero offset; use short, equal-length tubes.
- **Heater power**: measure 12 V and current with a clamp meter → P ≈ V·I at several PWM levels; note nonlinearity.

---

## 4) Logging & File Hygiene

- Log at least: `time, T_in, T_out, T_surface, PWM_fan, PWM_heater, dP, state, faults`.
- Save runs to `data/yyyy-mm-dd_testname.csv`.  
- Keep large logs out of Git history if they’re huge; consider **Git LFS** or add `data/` to `.gitignore`.

---

## 5) Control Tuning Notes

- Start with **PI** on heater loop; add small **D** only if overshoot persists.
- Use **soft-start**: ramp heater duty over 10–30 s to reduce overshoot.
- If oscillation occurs:
  - Reduce I-gain, add output **slew limit**, or increase fan flow for more **h**.
- Prefer **fixed airflow** for first tests; add supervisory fan control later.

---

## 6) Airflow & Leak Tests

- Prepare **interchangeable orifice inserts** (e.g., Ø2, 3, 4, 6, 8, 10 mm). Label each plate.
- For each orifice size: record steady **Δp**, estimated **Q**, warm-up time, steady power.
- Compare **single-hole + leak** vs **side vent** configurations under the same setpoint.

---

## 7) Common Pitfalls

- **Public repo ≠ write access**: collaborators need to be added on GitHub to push.
- **.gitignore not retroactive**: run `git rm -r --cached .` if you added it late.
- **BLDC fan flyback diode**: don’t add one; use **TVS** on the 12 V rail instead.
- **Noisy sensors** near heater cables: reroute, twist pairs, add small RC at MCU side.

---

## 8) Naming & Units (please stick to these)

- Temperatures: °C; Ambient `T_e`, Air `T_in`, `T_out`, Surface `T_s`.
- Power: W; Duty cycle: 0–100%.
- Pressure: Pa (Δp); Flow: m³/h (log both Δp and PWM; compute Q offline).
- Files: `sim/`, `control/`, `hardware/`, `docs/`, `data/`.

---

## 9) Branch & Changes (lightweight policy)

- Protect `main`; develop on feature branches.
- Use PRs with short descriptions: **why** + **what changed** + **how tested**.
- Tag key milestones (`v0.1-sim`, `v0.2-proto`).

---

## 10) TODO (next steps)

- [ ] Finalize exact Sunon fan P/N and add curve snapshot to `docs/`.
- [ ] Pick dP sensor range (±125 Pa vs ±500 Pa) and wire to I²C.
- [ ] Implement **state machine** (IDLE → PREHEAT → HOLD → FAULT).
- [ ] Add over-temp & sensor-timeout failsafes.
- [ ] Run first **setpoint sweep** (40, 50, 60 °C) and compare with simulation.

---

### Change Log

- **0.1** — Initial working notes added.
