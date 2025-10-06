# Two-point thermostat using Pico W internal temp sensor
# Servo on GP1. Servo power: 5V, GND shared with Pico W.

from machine import Pin, PWM, ADC
import time

# ===== Hardware =====
SERVO_PIN = 1
pwm = PWM(Pin(SERVO_PIN))
pwm.freq(50)   # 50 Hz PWM

# Servo pulse range (µs)
CLOSED_US = 1000   # gate fully closed
OPEN_US   = 2000   # gate fully open

def write_us(us):
    pwm.duty_u16(int(us * 65535 / 20000))  # 20 ms period at 50 Hz

# ===== Temperature sensor (internal) =====
adc = ADC(4)
VREF, ADC_MAX = 3.3, 65535

def read_temp_c():
    v = adc.read_u16() * VREF / ADC_MAX
    return 27.0 - (v - 0.706) / 0.001721

# ===== Control thresholds =====
T_LOW  = 19.0   # °C — below this, servo closes
T_HIGH = 20.0   # °C — above this, servo opens
# between T_LOW and T_HIGH → keep last state

# ===== Main loop =====
print("Two-point thermostat running. Ctrl+C to stop.")
servo_state = None  # None=unknown, True=open, False=closed

try:
    while True:
        t = read_temp_c()
        if t <= T_LOW and servo_state != False:
            write_us(CLOSED_US)
            servo_state = False
            print(f"T={t:.2f} °C → Servo CLOSED")
        elif t >= T_HIGH and servo_state != True:
            write_us(OPEN_US)
            servo_state = True
            print(f"T={t:.2f} °C → Servo OPEN")
        else:
            # hold last state
            print(f"T={t:.2f} °C → Servo HOLD ({'OPEN' if servo_state else 'CLOSED'})")
        time.sleep(1)

except KeyboardInterrupt:
    pass
finally:
    pwm.deinit()
    print("Stopped.")
