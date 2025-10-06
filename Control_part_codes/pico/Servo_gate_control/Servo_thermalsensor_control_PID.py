# Proportional-only temperature → servo control
# Servo signal: GP1 (PWM-capable). Servo power: 5V, shared GND with Pico W.

from machine import Pin, PWM, ADC
import time

# ===== Hardware =====
SERVO_PIN = 1
pwm = PWM(Pin(SERVO_PIN))
pwm.freq(50)   # 50 Hz standard for hobby servos

MIN_US, MAX_US = 1000, 2000   # servo range (0°..180° approx)

def write_us(us):
    pwm.duty_u16(int(us * 65535 / 20000))  # 20 ms period

# ===== Internal temp sensor =====
adc = ADC(4)
VREF = 3.3
ADC_MAX = 65535

def read_temp_c():
    v = adc.read_u16() * VREF / ADC_MAX
    return 27.0 - (v - 0.706) / 0.001721

# ===== Control params =====
SETPOINT = 30.0    # target temperature °C
Kp = 30.0          # proportional gain (servo µs per °C error)
DEADBAND = 5       # µs: ignore tiny changes to avoid jitter
UPDATE_PERIOD = 0.5  # seconds

# ===== Main loop =====
print("Proportional control running. Ctrl+C to stop.")
try:
    while True:
        temp = read_temp_c()
        error = SETPOINT - temp  # positive = too cold

        # Proportional output (center at midpoint)
        midpoint = (MIN_US + MAX_US) // 2
        pulse = int(midpoint + Kp * error)

        # Clamp to safe range
        pulse = max(MIN_US, min(MAX_US, pulse))

        # Deadband filter
        if abs(pulse - midpoint) > DEADBAND:
            write_us(pulse)

        print(f"T={temp:5.2f} °C  err={error:+5.2f}  pulse={pulse} us")
        time.sleep(UPDATE_PERIOD)

except KeyboardInterrupt:
    pass
finally:
    pwm.deinit()
    print("Stopped.")

