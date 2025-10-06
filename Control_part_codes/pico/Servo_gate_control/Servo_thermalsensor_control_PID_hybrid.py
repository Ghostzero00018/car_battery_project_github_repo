# Hybrid thermostat controller for Pico W
# Servo: GP1, powered from 5V (share GND with Pico W).

from machine import Pin, PWM, ADC
import time

# ===== Hardware =====
SERVO_PIN = 1
pwm = PWM(Pin(SERVO_PIN))
pwm.freq(50)   # 50 Hz standard servo frequency

CLOSED_US = 1000   # servo pulse width for closed gate (~0°)
OPEN_US   = 2000   # servo pulse width for open gate (~180°)

def write_us(us):
    pwm.duty_u16(int(us * 65535 / 20000))  # 20 ms period at 50 Hz

# ===== Internal temperature sensor =====
adc = ADC(4)
VREF, ADC_MAX = 3.3, 65535

def read_temp_c():
    v = adc.read_u16() * VREF / ADC_MAX
    return 27.0 - (v - 0.706) / 0.001721

# ===== Thresholds =====
T_LOW  = 18.0   # °C -> fully closed
T_HIGH = 22.0   # °C -> fully open

def temp_to_us(t):
    if t <= T_LOW:
        return CLOSED_US
    elif t >= T_HIGH:
        return OPEN_US
    else:
        # proportional mapping between T_LOW and T_HIGH
        frac = (t - T_LOW) / (T_HIGH - T_LOW)  # 0..1
        return int(CLOSED_US + frac * (OPEN_US - CLOSED_US))

# ===== Main loop =====
print("Hybrid thermostat running. Ctrl+C to stop.")

try:
    while True:
        t = read_temp_c()
        pulse = temp_to_us(t)
        write_us(pulse)
        print(f"T={t:.2f} °C → Servo pulse={pulse} us")
        time.sleep(1)

except KeyboardInterrupt:
    pass
finally:
    pwm.deinit()
    print("Stopped.")
