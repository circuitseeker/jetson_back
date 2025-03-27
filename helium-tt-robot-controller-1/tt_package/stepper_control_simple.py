#!/usr/bin/env python3
import gpiod
import time
import math
import signal
import sys

class GPIOControl:
    def __init__(self):
        # Define GPIO pins
        self.pins = {
            'ROT_DIR': 138,    # PAC.00
            'ROT_STEP': 144,   # PAC.06
            'MOVE_DIR': 122,   # PY.00
            'MOVE_STEP': 123   # PY.01
        }
        self.chip = None
        self.lines = {}
        self.setup_gpio()
        
    def setup_gpio(self):
        """Setup GPIO using gpiod"""
        try:
            print("Setting up GPIO...")
            # Open GPIO chip 0
            self.chip = gpiod.Chip('gpiochip0')
            
            # Request GPIO lines
            for name, pin in self.pins.items():
                try:
                    line = self.chip.get_line(pin)
                    config = gpiod.LineRequest()
                    config.consumer = "stepper_control"
                    config.request_type = gpiod.LineRequest.DIRECTION_OUTPUT
                    line.request(config)
                    self.lines[name] = line
                    print(f"Successfully configured GPIO{pin} for {name}")
                except Exception as e:
                    print(f"Error setting up GPIO{pin} for {name}: {e}")
                    self.cleanup()
                    raise
                    
            print("GPIO setup completed successfully")
        except Exception as e:
            print(f"GPIO setup error: {e}")
            raise
            
    def write_pin(self, name, value):
        """Write a value to a GPIO pin"""
        try:
            self.lines[name].set_value(value)
        except Exception as e:
            print(f"Error writing to {name} (GPIO{self.pins[name]}): {e}")
            raise
            
    def cleanup(self):
        """Release all GPIO lines"""
        print("Cleaning up GPIO...")
        for line in self.lines.values():
            try:
                line.release()
            except:
                pass
        if self.chip:
            try:
                self.chip.close()
            except:
                pass
        print("GPIO cleanup completed")

# ----------- Stepper Motor Settings -----------
ROT_STEPS_PER_REV = 400
ROT_MICROSTEPS = 32
MOVE_STEPS_PER_REV = 400
MOVE_MICROSTEPS = 32
STEPS_PER_CM = 200

# Motor control parameters
MIN_STEP_DELAY = 0.0001
MAX_STEP_DELAY = 0.001
ACCELERATION = 0.00001

def calculate_step_delay(current_step, total_steps):
    """Calculate step delay based on acceleration profile"""
    if current_step < total_steps * 0.2:  # Acceleration phase
        delay = MAX_STEP_DELAY - (current_step * ACCELERATION)
    elif current_step > total_steps * 0.8:  # Deceleration phase
        delay = MAX_STEP_DELAY - ((total_steps - current_step) * ACCELERATION)
    else:  # Constant speed phase
        delay = MIN_STEP_DELAY
    return max(MIN_STEP_DELAY, min(MAX_STEP_DELAY, delay))

def step_motor(gpio, dir_name, step_name, direction, steps):
    """Step motor a specified number of steps with acceleration control"""
    try:
        print(f"Moving motor: direction={direction}, steps={steps}")
        gpio.write_pin(dir_name, 1 if direction else 0)
        time.sleep(0.0001)  # Direction setup time
        
        for i in range(steps):
            delay = calculate_step_delay(i, steps)
            
            gpio.write_pin(step_name, 1)
            time.sleep(delay)
            gpio.write_pin(step_name, 0)
            time.sleep(delay)
            
            if (i + 1) % 1000 == 0:
                print(f"Completed {i + 1}/{steps} steps (delay: {delay:.6f}s)")
            
    except Exception as e:
        print(f"Step error: {e}")
        gpio.write_pin(step_name, 0)
        raise

def rotate(gpio, angle):
    """Rotate to specified angle (in degrees)"""
    try:
        steps = int((ROT_STEPS_PER_REV * ROT_MICROSTEPS * angle) / 360)
        direction = steps > 0
        steps = abs(steps)
        
        print(f"\nRotation Details:")
        print(f"Angle: {angle} degrees")
        print(f"Steps per revolution: {ROT_STEPS_PER_REV}")
        print(f"Microsteps: {ROT_MICROSTEPS}")
        print(f"Total steps: {steps}")
        print(f"Direction: {'Clockwise' if direction else 'Counter-clockwise'}")
        
        step_motor(gpio, 'ROT_DIR', 'ROT_STEP', direction, steps)
    except Exception as e:
        print(f"Rotation error: {e}")

def move(gpio, distance_cm):
    """Move specified distance (in cm)"""
    try:
        if distance_cm == 0:
            return
            
        steps = int(STEPS_PER_CM * MOVE_MICROSTEPS * distance_cm)
        direction = steps > 0
        steps = abs(steps)
        
        print(f"\nMovement Details:")
        print(f"Distance: {distance_cm} cm")
        print(f"Steps per cm: {STEPS_PER_CM}")
        print(f"Microsteps: {MOVE_MICROSTEPS}")
        print(f"Total steps: {steps}")
        print(f"Direction: {'Forward' if direction else 'Backward'}")
        
        step_motor(gpio, 'MOVE_DIR', 'MOVE_STEP', direction, steps)
    except Exception as e:
        print(f"Movement error: {e}")

def signal_handler(signum, frame):
    """Handle cleanup on program termination"""
    print("\nReceived signal to terminate...")
    sys.exit(0)

def main():
    gpio = None
    try:
        # Register signal handlers
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Initialize GPIO
        gpio = GPIOControl()
        
        print("\nStepper Motor Control")
        print("--------------------")
        print(f"Rotation Motor:")
        print(f"  Steps per revolution: {ROT_STEPS_PER_REV}")
        print(f"  Microsteps: {ROT_MICROSTEPS}")
        print(f"  Total steps per revolution: {ROT_STEPS_PER_REV * ROT_MICROSTEPS}")
        print(f"\nMovement Motor:")
        print(f"  Steps per revolution: {MOVE_STEPS_PER_REV}")
        print(f"  Microsteps: {MOVE_MICROSTEPS}")
        print(f"  Steps per cm: {STEPS_PER_CM}")
        print(f"  Total steps per cm: {STEPS_PER_CM * MOVE_MICROSTEPS}")
        print("\nAcceleration Settings:")
        print(f"  Minimum step delay: {MIN_STEP_DELAY:.6f}s")
        print(f"  Maximum step delay: {MAX_STEP_DELAY:.6f}s")
        print(f"  Acceleration factor: {ACCELERATION:.6f}")
        print("\nEnter commands in the format: <rotation_angle>,<distance_in_cm>")
        print("Example: 90,20 (rotate 90 degrees, move 20 cm)")
        print("Press Ctrl+C to exit\n")
        
        while True:
            try:
                command = input("Enter command (angle,distance): ")
                angle, distance = map(int, command.split(','))
                
                rotate(gpio, angle)
                move(gpio, distance)
                print("Commands completed")
                
            except ValueError:
                print("Invalid input. Please use format: angle,distance")
            except KeyboardInterrupt:
                print("\nExiting...")
                break
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if gpio:
            gpio.cleanup()

if __name__ == "__main__":
    main() 