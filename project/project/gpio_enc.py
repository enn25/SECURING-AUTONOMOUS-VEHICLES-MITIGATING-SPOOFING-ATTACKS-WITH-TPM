from gpiozero import Motor, PWMOutputDevice, RotaryEncoder
from time import sleep
import threading
import time

# Define Motors
motorA = Motor(forward=17, backward=18)  # Left Motor
motorB = Motor(forward=22, backward=23)  # Right Motor

# Enable PWM for Speed Control
speedA = PWMOutputDevice(13)  # ENA
speedB = PWMOutputDevice(12)  # ENB

# Define Encoders
encoderA = RotaryEncoder(6, 5)
encoderB = RotaryEncoder(24, 25)

# Use a lock for thread safety
encoder_lock = threading.Lock()
stop_flag = threading.Event()

def monitor_encoders():
    """Monitor encoder counts in a separate thread"""
    while not stop_flag.is_set():
        with encoder_lock:
            count_a = encoderA.steps
            count_b = encoderB.steps
            
        print(f"Encoder A: {count_a} pulses | Encoder B: {count_b} pulses")
        time.sleep(0.5)  # Update every 0.5 seconds

def reset_encoders():
    """Safely reset both encoders"""
    with encoder_lock:
        encoderA.steps = 0
        encoderB.steps = 0

def get_encoder_values():
    """Safely get current encoder values"""
    with encoder_lock:
        return encoderA.steps, encoderB.steps

def test_encoder_run(duration=10):
    """Run motors forward and backward to test encoder functionality"""
    reset_encoders()
    print("Testing encoder response - Forward movement")
    
    # Set both motors to full speed
    motorA.forward()
    motorB.forward()
    speedA.value = 1.0
    speedB.value = 1.0
    
    start_time = time.time()
    while time.time() - start_time < duration:
        count_a, count_b = get_encoder_values()
        print(f"Time: {time.time() - start_time:.1f}s - Encoder A: {count_a} | Encoder B: {count_b}")
        sleep(0.5)
    
    # Stop motors
    motorA.stop()
    motorB.stop()
    sleep(1)
    
    # Test backward movement
    reset_encoders()
    print("Testing encoder response - Backward movement")
    
    motorA.backward()
    motorB.backward()
    speedA.value = 1.0
    speedB.value = 1.0
    
    start_time = time.time()
    while time.time() - start_time < duration:
        count_a, count_b = get_encoder_values()
        print(f"Time: {time.time() - start_time:.1f}s - Encoder A: {count_a} | Encoder B: {count_b}")
        sleep(0.5)
    
    # Stop motors
    motorA.stop()
    motorB.stop()

try:
    # Start monitoring thread
    stop_flag.clear()
    monitor_thread = threading.Thread(target=monitor_encoders)
    monitor_thread.daemon = True
    monitor_thread.start()
    sleep(1)  # Give thread time to start

    print("=== ENCODER TEST STARTED ===")
    
    # Forward movement test
    reset_encoders()
    print("Moving forward at full speed")
    motorA.forward()
    motorB.forward()
    speedA.value = 1.0
    speedB.value = 1.0
    sleep(5)
    count_a, count_b = get_encoder_values()
    print(f"Forward movement complete - Encoder A: {count_a} | Encoder B: {count_b}")

    # Left turn (differential steering - slow down right motor)
    reset_encoders()
    print("Turning left (differential steering)")
    motorA.forward()
    motorB.forward()
    speedA.value = 1.0    # Left motor full speed
    speedB.value = 0.1    # Right motor reduced speed
    sleep(5)
    count_a, count_b = get_encoder_values()
    print(f"Left turn complete - Encoder A: {count_a} | Encoder B: {count_b}")

    # Right turn (differential steering - slow down left motor)
    reset_encoders()
    print("Turning right (differential steering)")
    motorA.forward()
    motorB.forward()
    speedA.value = 0.1    # Left motor reduced speed
    speedB.value = 1.0    # Right motor full speed
    sleep(5)
    count_a, count_b = get_encoder_values()
    print(f"Right turn complete - Encoder A: {count_a} | Encoder B: {count_b}")

    # Stop
    motorA.stop()
    motorB.stop()
    sleep(1)
    
    # Comprehensive encoder test
    print("\n=== COMPREHENSIVE ENCODER TEST ===")
    test_encoder_run(duration=15)
    
    # Summary
    print("\n=== ENCODER TEST SUMMARY ===")
    count_a, count_b = get_encoder_values()
    print(f"Final encoder readings - A: {count_a}, B: {count_b}")
    print("If encoders show progressively changing values during movement, your connections are working.")
    print("If values jump or remain static, there may be wiring issues or encoder problems.")

except KeyboardInterrupt:
    print("Interrupted. Stopping motors.")

finally:
    # Clean up
    stop_flag.set()
    motorA.stop()
    motorB.stop()
    speedA.close()
    speedB.close()
    sleep(1)  # Give monitoring thread time to stop
    print("Test complete. Resources released.")
