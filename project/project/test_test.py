from gpiozero import RotaryEncoder
import time

encoderA = RotaryEncoder(6, 5, max_steps=0)   # Left Wheel Encoder
encoderB = RotaryEncoder(24, 25, max_steps=0)  # Right Wheel Encoder

while True:
    print(f"Encoder A: {encoderA.steps}, Encoder B: {encoderB.steps}")
    time.sleep(0.1)
