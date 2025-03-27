#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time
import threading
import os
import signal

# ----------- Stepper Motor 1 (Rotation) Settings -----------
ROT_DIR_PIN = 7    # GPIO7 for rotation direction
ROT_STEP_PIN = 11  # GPIO11 for rotation step
ROT_STEPS_PER_REV = 200  # Standard NEMA 23 steps per revolution
ROT_MICROSTEPS = 16  # Microstepping setting (1, 2, 4, 8, 16, 32)

# Define speed and acceleration for stepper motor 1
ROT_MAX_SPEED = 1000  # steps per second (adjusted for NEMA 23)
ROT_ACCELERATION = 500  # steps per second squared

# ----------- Stepper Motor 2 (Pull/Push) Settings -----------
MOVE_DIR_PIN = 13    # GPIO13 for movement direction
MOVE_STEP_PIN = 15   # GPIO15 for movement step
MOVE_STEPS_PER_REV = 200  # Standard NEMA 23 steps per revolution
MOVE_MICROSTEPS = 16  # Microstepping setting (1, 2, 4, 8, 16, 32)

# Conversion factor: number of steps per cm of movement
STEPS_PER_CM = 100  # Adjust this value based on your drive mechanism

# Increased speed and acceleration for stepper motor 2
MOVE_MAX_SPEED = 1000  # steps per second (adjusted for NEMA 23)
MOVE_ACCELERATION = 500  # steps per second squared

# Variables for motor control
rot_running = False
move_running = False
cleanup_done = False

def signal_handler(signum, frame):
    """Handle cleanup on program termination"""
    global cleanup_done
    if not cleanup_done:
        force_cleanup()
        cleanup_done = True
    exit(0)

def force_cleanup():
    """Force cleanup of GPIO resources"""
    global cleanup_done
    if cleanup_done:
        return
        
    try:
        # First try normal cleanup
        GPIO.cleanup()
    except:
        pass
        
    try:
        # Additional cleanup using system commands
        os.system('sudo chmod 666 /dev/gpiochip0')
        os.system('sudo chmod 666 /dev/gpiochip1')
        
        # Kill any Python processes that might be using GPIO
        os.system('sudo pkill -f "python3.*stepper_control"')
        
        # Wait a moment for resources to be released
        time.sleep(0.5)
        
        cleanup_done = True
    except Exception as e:
        print(f"Cleanup error: {e}")

def setup_gpio():
    """Setup GPIO with error handling"""
    global cleanup_done
    
    try:
        # Force cleanup first
        force_cleanup()
        
        # Set GPIO mode to BCM
        GPIO.setmode(GPIO.BOARD)
        
        # Setup pins with error handling
        for pin in [ROT_DIR_PIN, ROT_STEP_PIN, MOVE_DIR_PIN, MOVE_STEP_PIN]:
            try:
                GPIO.setup(pin, GPIO.OUT)
                # Initialize pins to LOW state
                GPIO.output(pin, GPIO.LOW)
            except Exception as e:
                print(f"Error setting up pin {pin}: {e}")
                force_cleanup()
                raise
        
        # Wait a moment for GPIO to stabilize
        time.sleep(0.1)
        
        print("GPIO setup successful")
    except Exception as e:
        print(f"GPIO setup error: {e}")
        force_cleanup()
        raise

class StepperController:
    def __init__(self, dir_pin, step_pin, steps_per_rev, microsteps, max_speed, acceleration):
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.steps_per_rev = steps_per_rev
        self.microsteps = microsteps
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.current_position = 0
        self.target_position = 0
        self.running = False
        self.current_speed = 0
        
    def step(self, direction):
        """Perform a single step"""
        try:
            GPIO.output(self.dir_pin, GPIO.HIGH if direction else GPIO.LOW)
            time.sleep(0.0001)  # Direction setup time
            
            GPIO.output(self.step_pin, GPIO.HIGH)
            time.sleep(0.0002)  # Step pulse width
            GPIO.output(self.step_pin, GPIO.LOW)
            time.sleep(0.0002)  # Minimum time between pulses
        except Exception as e:
            print(f"Step error: {e}")
            self.stop()
            raise
        
    def move_to(self, position):
        """Move to absolute position"""
        try:
            self.target_position = position
            steps = position - self.current_position
            if steps != 0:
                direction = steps > 0
                steps = abs(steps)
                
                # Calculate acceleration profile
                steps_to_accel = int((self.max_speed ** 2) / (2 * self.acceleration))
                if steps > 2 * steps_to_accel:
                    # Full acceleration profile
                    for i in range(steps):
                        if i < steps_to_accel:
                            # Acceleration phase
                            self.current_speed = int((2 * self.acceleration * i) ** 0.5)
                        elif i > steps - steps_to_accel:
                            # Deceleration phase
                            self.current_speed = int((2 * self.acceleration * (steps - i)) ** 0.5)
                        else:
                            # Constant speed phase
                            self.current_speed = self.max_speed
                            
                        self.step(direction)
                        time.sleep(1.0 / (self.current_speed * self.microsteps))
                else:
                    # Partial acceleration profile
                    for i in range(steps):
                        if i < steps // 2:
                            # Acceleration phase
                            self.current_speed = int((2 * self.acceleration * i) ** 0.5)
                        else:
                            # Deceleration phase
                            self.current_speed = int((2 * self.acceleration * (steps - i)) ** 0.5)
                        self.step(direction)
                        time.sleep(1.0 / (self.current_speed * self.microsteps))
                
                self.current_position = position
        except Exception as e:
            print(f"Move error: {e}")
            self.stop()
            raise
            
    def move(self, distance):
        """Move relative to current position"""
        self.move_to(self.current_position + distance)
        
    def stop(self):
        """Stop the motor"""
        self.running = False
        try:
            GPIO.output(self.step_pin, GPIO.LOW)
        except:
            pass

def rotation_thread():
    """Thread to handle rotation motor movement"""
    global rot_running
    
    while True:
        try:
            if rot_running:
                rot_controller.run()
            time.sleep(0.001)
        except Exception as e:
            print(f"Rotation thread error: {e}")
            rot_running = False
            force_cleanup()
            break

def movement_thread():
    """Thread to handle movement motor control"""
    global move_running
    
    while True:
        try:
            if move_running:
                move_controller.run()
            time.sleep(0.001)
        except Exception as e:
            print(f"Movement thread error: {e}")
            move_running = False
            force_cleanup()
            break

def start_rotation(angle):
    """Start rotation motor to a specific angle (in degrees)"""
    global rot_running
    try:
        # Calculate steps needed for the angle, accounting for microstepping
        target_steps = int((ROT_STEPS_PER_REV * ROT_MICROSTEPS * angle) / 360)
        rot_controller.move_to(target_steps)
        rot_running = True
    except Exception as e:
        print(f"Rotation start error: {e}")
        rot_running = False

def start_movement(distance_cm):
    """Start movement motor to move a specified distance (in cm)"""
    global move_running
    try:
        if distance_cm == 0:
            return
        # Calculate steps needed for the distance, accounting for microstepping
        target_steps = int(STEPS_PER_CM * MOVE_MICROSTEPS * distance_cm)
        move_controller.move_to(target_steps)
        move_running = True
    except Exception as e:
        print(f"Movement start error: {e}")
        move_running = False

def main():
    global rot_controller, move_controller
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Force cleanup before starting
        force_cleanup()
        
        # Setup GPIO
        setup_gpio()
        
        # Create stepper controllers
        rot_controller = StepperController(
            ROT_DIR_PIN, ROT_STEP_PIN, ROT_STEPS_PER_REV,
            ROT_MICROSTEPS, ROT_MAX_SPEED, ROT_ACCELERATION
        )
        
        move_controller = StepperController(
            MOVE_DIR_PIN, MOVE_STEP_PIN, MOVE_STEPS_PER_REV,
            MOVE_MICROSTEPS, MOVE_MAX_SPEED, MOVE_ACCELERATION
        )
        
        # Create and start control threads
        rot_thread = threading.Thread(target=rotation_thread, daemon=True)
        move_thread = threading.Thread(target=movement_thread, daemon=True)
        rot_thread.start()
        move_thread.start()
        
        print("Ready. Send commands in the format: <rotation_angle>,<distance_in_cm>")
        print(f"Using {ROT_MICROSTEPS} microsteps for rotation motor")
        print(f"Using {MOVE_MICROSTEPS} microsteps for movement motor")
        
        # Main loop for command processing
        while True:
            try:
                command = input("Enter command (angle,distance): ")
                angle, distance = map(int, command.split(','))
                start_rotation(angle)
                start_movement(distance)
                print("Commands started")
            except ValueError:
                print("Invalid input. Please use format: angle,distance")
            except KeyboardInterrupt:
                break
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        force_cleanup()

if __name__ == "__main__":
    main() 