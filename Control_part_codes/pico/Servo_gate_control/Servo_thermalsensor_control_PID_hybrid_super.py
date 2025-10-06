# Pico W — Robust Hybrid Thermostat → Servo (GP1)
# Power servo from 5V, share GND with Pico. No Wi-Fi required.

from machine import Pin, PWM, ADC
import time

# ========= USER SETTINGS =========
# Pins/servo geometry
SERVO_PIN       = 1          # PWM pin to servo signal
SERVO_FREQ_HZ   = 50         # hobby servos use ~50 Hz
CLOSED_US       = 1000       # ~0°
OPEN_US         = 2000       # ~180°
REVERSED        = False      # True to invert motion

# Temp thresholds (°C)
T_LOW           = 18.0       # ≤ T_LOW  → fully CLOSED
T_HIGH          = 22.0       # ≥ T_HIGH → fully OPEN

# Robustness
TEMP_ALPHA      = 0.25       # EMA smoothing for temperature (0..1)
EDGE_HYST       = 0.5        # extra °C stickiness at edges (anti-chatter)
DEADBAND_US     = 6          # ignore tiny pulse changes ≤ this
RAMP_US_PER_S   = 400        # limit speed (µs per second) for gentle motion
WARMUP_S        = 2.0        # ignore first seconds to stabilize ADC

# Loop timing & telemetry
UPDATE_HZ       = 10         # control loop rate
PRINT_EVERY_S   = 1.0        # telemetry interval

# Simple temperature calibration (optional)
TEMP_OFFSET_C   = 0.0        # add after conversion
TEMP_SCALE      = 1.0        # multiply after offset
# =================================


# ---- Low-level helpers ----
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def lerp(a, b, x):
    return a + (b - a) * x


# ---- Servo setup ----
servo = PWM(Pin(SERVO_PIN))
servo.freq(SERVO_FREQ_HZ)
_period_us = 1_000_000 // SERVO_FREQ_HZ  # 20_000 for 50 Hz

def _apply_reverse(us):
    if not REVERSED:
        return us
    # Mirror around mid
    mid = (CLOSED_US + OPEN_US) // 2
    return mid - (us - mid)

def write_us(us):
    us = clamp(us, min(CLOSED_US, OPEN_US), max(CLOSED_US, OPEN_US))
    us = _apply_reverse(us)
    servo.duty_u16(int(us * 65535 // _period_us))


# ---- Temperature (internal sensor on ADC4) ----
adc = ADC(4)
VREF, ADC_MAX = 3.3, 65535

def read_temp_c_raw():
    v = adc.read_u16() * VREF / ADC_MAX
    # RP2040 formula: T = 27 - (V - 0.706) / 0.001721
    return 27.0 - (v - 0.706) / 0.001721

def read_temp_c():
    t = read_temp_c_raw()
    t = (t + TEMP_OFFSET_C) * TEMP_SCALE
    return t


# ---- Hybrid mapping with hysteresis & smoothing ----
def temp_to_target_us(t):
    # Add a small hysteresis band around edges
    if t <= (T_LOW - EDGE_HYST):
        return CLOSED_US
    if t >= (T_HIGH + EDGE_HYST):
        return OPEN_US
    # Proportional zone between thresholds (clamped to [0..1])
    x = clamp((t - T_LOW) / (T_HIGH - T_LOW), 0.0, 1.0)
    return int(lerp(CLOSED_US, OPEN_US, x))


# ---- Main control loop ----
dt = 1.0 / UPDATE_HZ
failsafe = (CLOSED_US + OPEN_US) // 2
tele_next = time.ticks_ms()

print("Hybrid thermostat (robust) running. Ctrl+C to stop.")
# Warmup + initial temperature EMA
t_avg = read_temp_c()
write_us(failsafe)

# Warm-up period (optional)
t0 = time.ticks_ms()
while (time.ticks_ms() - t0) < int(WARMUP_S * 1000):
    t_avg = TEMP_ALPHA * read_temp_c() + (1.0 - TEMP_ALPHA) * t_avg
    time.sleep(0.05)

# Start at mapped target
current_us = temp_to_target_us(t_avg)
write_us(current_us)

try:
    while True:
        loop_start = time.ticks_ms()

        # Read & smooth temperature
        t_now = read_temp_c()
        t_avg = TEMP_ALPHA * t_now + (1.0 - TEMP_ALPHA) * t_avg

        # Compute target and limit slew rate
        target_us = temp_to_target_us(t_avg)
        max_step = int(RAMP_US_PER_S * dt)
        if target_us > current_us + max_step:
            target_us = current_us + max_step
        elif target_us < current_us - max_step:
            target_us = current_us - max_step

        # Deadband to reduce unnecessary writes
        if abs(target_us - current_us) > DEADBAND_US:
            write_us(target_us)
            current_us = target_us

        # Periodic telemetry
        if time.ticks_diff(time.ticks_ms(), tele_next) >= 0:
            print(f"T={t_avg:5.2f} °C  pulse={current_us} us  "
                  f"zone={'CLOSED' if current_us<=CLOSED_US else 'OPEN' if current_us>=OPEN_US else 'MIX'}")
            tele_next = time.ticks_add(tele_next, int(PRINT_EVERY_S * 1000))

        # Pace the loop
        elapsed = time.ticks_diff(time.ticks_ms(), loop_start) / 1000.0
        sleep_left = dt - elapsed
        if sleep_left > 0:
            time.sleep(sleep_left)

except KeyboardInterrupt:
    pass
finally:
    servo.deinit()
    print("Stopped.")
