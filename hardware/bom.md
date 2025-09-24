# Bill of Materials (BoM) — Car Battery Thermal Control Prototype

Version: 0.1 (draft)  
Scope: Hot-box prototype for battery thermal conditioning with **Raspberry Pi Pico W** control, **12 V Sunon fan**, heater, and temperature/pressure sensing.

---

## 1) Electronics (Control & Power)

| Group | Qty | Part / Model | Key Specs | Notes |
|---|---:|---|---|---|
| Microcontroller | 1 | **Raspberry Pi Pico W** | RP2040, 2.4 GHz Wi-Fi, 3.3 V I/O | Main controller (PWM fan + heater, sensor IO). Include pin headers if needed. |
| Power Supply | 1 | 12 V DC PSU | ≥ **5 A** (headroom), barrel or screw-terminals | Powers fan + heater (and 5 V/3.3 V rails via buck/USB). Choose higher current if heater >50 W. |
| Buck Converter | 1 | 12 V → 5 V DC-DC | ≥ **2 A**, ripple <50 mV | For sensors/logic if not using Pico’s USB as 5 V source. |
| Heater Switch (MOSFET) | 1 | **IRLZ44N** (logic-level N-MOSFET) or **AOI518**/**AOD510** | V_DS ≥ 30 V, I_D ≥ 30 A, R_DS(on) ≤ 20 mΩ @ 4.5 V | Low-side PWM of heater. Add small heatsink/pad; gate via 100 Ω; include gate-to-GND 100 kΩ. |
| Fan Switch (MOSFET) | 1 | Logic-level N-MOSFET (e.g., **IRLZ44N**) | As above | Low-side PWM enable/speed control for 12 V BLDC fan. |
| TVS Protection | 2 | **SMBJ18A** (or 600 W 18 V SMAJ) | 18 V standoff | Across 12 V rail near fan & heater to clamp transients. |
| Flyback / Snubbers | 0–2 | RC Snubber (optional) | e.g., 100 Ω + 100 nF (X7R) | BLDC fans have internal drivers; avoid diode across fan. Use TVS/snubber on 12 V line if noise observed. |
| Gate Resistor | 2 | 100 Ω | 0.25 W | Between Pico GPIO and each MOSFET gate. |
| Gate Pulldown | 2 | 100 kΩ | 0.25 W | Gate-to-GND, keeps MOSFET off at boot. |
| Current Sense (opt.) | 1 | Low-side shunt + amp (e.g., INA219/INA180) | 0–3 A or per load | For logging heater/fan current. Optional. |
| Fuse | 2 | Blade fuse holders + fuses | **Fan line**: 2 A; **Heater line**: per power | Place on 12 V distribution before each load. |
| Connectors | — | 2-pin/3-pin terminal blocks (5.08 mm), JST-VH/KK | 12 V lines, sensors | Use locking connectors for vibration safety. |
| Wiring | — | 16–20 AWG (power), 24–26 AWG (signals) | Silicone-insulated preferred | Keep power and sensor harness separated to reduce noise. |
| PCB/Proto | 1 | Perfboard or small PCB | — | For MOSFETs, TVS, connectors. Add copper pour for dissipation. |
| Heatsink/Pad | 1–2 | Small TO-220 heatsink + thermal pad | θ_sa ≤ 30 °C/W | For MOSFETs if heater power is high / enclosure warm. |

---

## 2) Actuators (Fan & Heater)

| Group | Qty | Part / Model | Key Specs | Notes |
|---|---:|---|---|---|
| Fan (Primary) | 1 | **Sunon 60×60×38 mm, 12 V** (≈ **96 m³/h**) | BLDC, 10–12 W class | Matches your earlier choice. Verify exact P/N against available stock/curve. |
| Fan Grill + Screws | 1 | 60 mm metal grill + M4 kit | — | Safety + airflow. |
| Heater | 1 | 12–24 V **resistive heater** (cartridge or wirewound) | Power: per simulation (e.g., **50–120 W**); temp rating ≥ 200 °C | Choose voltage to match PSU. If using 24 V heater, supply must be 24 V or use separate PSU. |
| Thermal Fuse | 1 | Non-resettable thermal cutoff | Trip 10–15 °C above max allowed | In series with heater for fail-safe. |
| Thermal Pad/Tape | 1 | High-temp mounting | ≥ 150 °C | For attaching cartridge/wire heater to plate if needed. |

> **Heater sizing**: From sim, ensure ≥30% headroom over steady-state power. Prefer PWM control via MOSFET over linear control.

---

## 3) Sensors

| Group | Qty | Part / Model | Key Specs | Notes |
|---|---:|---|---|---|
| Air Temp (inlet) | 1 | **DS18B20** (digital) or 10 k NTC | −10…85 °C (DS18B20) | DS18B20 (OneWire) simplifies wiring; NTC gives faster response but needs ADC. |
| Air Temp (outlet) | 1 | **DS18B20** or 10 k NTC | As above | For ΔT across box. |
| Battery Surface Temp | 1 | **DS18B20 TO-92** with thermal epoxy **or** NTC epoxied | −10…125 °C | Bond to surface; strain-relief the lead. |
| Differential Pressure | 1 | **Low-range dP sensor** (e.g., **SDP31/SDP810**, Honeywell **ABP** series) | ±125 Pa … ±500 Pa range, I²C/analog | Select range to match expected duct/box Δp; tube barbs required. |
| Ambient Temp (opt.) | 1 | **BME280/HTU21** | T/H (and P) | For ambient compensation and logs. |
| Tubing & Ports | 1 | 3–4 mm ID silicone + panel barbs | — | For dP across orifice/box; keep tubes short and equal. |

**Note:** If you choose NTCs, add: pull-down resistors (e.g., 10 k 1%), filtering (RC), and calibrate beta values.

---

## 4) Enclosure & Mechanics

| Group | Qty | Part / Model | Key Specs | Notes |
|---|---:|---|---|---|
| Hot Box | 1 | Insulated box (DIY or off-the-shelf) | Heat-resistant interior | Must tolerate heater proximity; add standoffs for test article. |
| Orifice / Leak Kit | 1 | Interchangeable inserts (drilled plates) | Ø 2–10 mm set | For **single-hole + leak** tests; label diameters. |
| Cable Glands | 2–4 | M12/M16 | For 3–6 mm cable | Feed-throughs for sensors and power. |
| Standoffs | 4 | Nylon/ceramic | — | Thermal isolation of DUT from walls. |
| Mounting Plate | 1 | Aluminum or FR-4 | — | For heater + temp sensor interface in box. |
| Fan Gasket | 1 | Foam or silicone | — | Reduces recirculation/leaks around fan frame. |

---

## 5) Tools & Consumables (not shipped with design)

- Soldering iron, leaded solder (or lead-free), flux
- Crimp tool (for JST/KK) and assortment of **ferrules**
- Heat-shrink tubing, zip ties, **PTFE** insulated wire for high-temp sections
- Thermal epoxy for sensor bonding
- Multimeter, clamp meter (optional), handheld manometer (optional for dP sanity check)

---

## 6) Electrical Ratings & Budgets (example)

| Load | Voltage | Power / Current | Notes |
|---|---:|---:|---|
| Fan | 12 V | 10–12 W (~0.9 A peak) | Depends on model/PWM. |
| Heater | 12 V | 60 W ⇒ **5 A** (example) | Size to your setpoint/ambient. |
| Pico + Sensors | 5 V | ≤ **300 mA** | Buck from 12 V if not using USB 5 V. |

> **PSU sizing rule**: Sum currents × 1.5 safety factor.

---

## 7) Pico W Pinout (proposed)

| Function | Pico W Pin | Notes |
|---|---|---|
| Heater PWM (MOSFET gate) | **GP15** | With 100 Ω series; 100 kΩ pulldown. |
| Fan PWM (MOSFET gate) | **GP14** | As above. |
| DS18B20 OneWire | **GP4** | 4.7 kΩ pull-up to 3.3 V. |
| I²C (dP sensor) | **GP6 (SDA), GP7 (SCL)** | 4.7 kΩ pull-ups to 3.3 V if needed. |
| Spare ADC (NTC alt.) | **GP26/27/28** | RC filter if long leads. |
| UART (logging, opt.) | **GP0/GP1** | To USB-UART if desired. |
| GND / 3V3 | — | Common grounds between logic and power. |

---

## 8) Part Number Suggestions (fill when ordering)

| Item | Preferred P/N | Alt P/N | Vendor/Link | Status |
|---|---|---|---|---|
| Sunon fan 60×60×38 mm, 12 V | *(fill exact P/N)* | — | — | ☐ |
| Heater (XX W @ YY V) | — | — | — | ☐ |
| dP sensor (±ZZ Pa) | — | — | — | ☐ |
| Pico W | RPI-PICO-W | — | — | ☐ |
| MOSFET (heater) | IRLZ44N | AOI518 / AOD510 | — | ☐ |
| TVS 18 V | SMBJ18A | SMAJ18A | — | ☐ |
| Buck 12→5 V | — | — | — | ☐ |
| DS18B20 (×3) | DS18B20+ | — | — | ☐ |
| Fuse holders + fuses | — | — | — | ☐ |
| Terminal blocks | — | — | — | ☐ |

---

## 9) Safety Notes

- Place a **fuse** on each 12 V branch (fan, heater).  
- Add a **thermal cutoff** in series with the heater.  
- Keep **sensor wiring** away from **heater power** lines; twist pairs for dP tubing if long.  
- Verify **MOSFET temperature** under worst-case duty cycle; heatsink if needed.  
- Use **TVS** on the 12 V rail near loads; avoid flyback diodes across BLDC fans.

---

### Change Log

- **0.1** — Initial draft BoM for simulation prototype.
