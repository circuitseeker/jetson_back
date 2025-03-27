#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time
import math

class AccelStepper:
    def __init__(self, dir_pin, step_pin, max_speed, acceleration, microsteps=16):
        """
        Initialize AccelStepper with pin configuration and motion parameters
        
        Args:
            dir_pin (int): GPIO pin number for direction control
            step_pin (int): GPIO pin number for step control
            max_speed (float): Maximum speed in steps per second
            acceleration (float): Acceleration in steps per second squared
            microsteps (int): Number of microsteps per step (default: 16)
        """
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.microsteps = microsteps
        
        # Motion control variables
        self.current_speed = 0
        self.current_position = 0
        self.target_position = 0
        self.running = False
        self.last_step_time = 0
        self.step_interval = 0
        
        # Setup GPIO pins
        try:
            GPIO.setup(self.dir_pin, GPIO.OUT)
            GPIO.setup(self.step_pin, GPIO.OUT)
            GPIO.output(self.step_pin, GPIO.LOW)
            GPIO.output(self.dir_pin, GPIO.LOW)
        except Exception as e:
            print(f"GPIO setup error in AccelStepper: {e}")
            raise
        
    def set_target_position(self, position):
        """
        Set the target position for the stepper motor
        
        Args:
            position (int): Target position in steps
        """
        self.target_position = position
        self.running = True
        
    def set_speed(self, speed):
        """
        Set the maximum speed for the stepper motor
        
        Args:
            speed (float): Maximum speed in steps per second
        """
        self.max_speed = speed
        
    def set_acceleration(self, acceleration):
        """
        Set the acceleration for the stepper motor
        
        Args:
            acceleration (float): Acceleration in steps per second squared
        """
        self.acceleration = acceleration
        
    def get_current_position(self):
        """
        Get the current position of the stepper motor
        
        Returns:
            int: Current position in steps
        """
        return self.current_position
        
    def get_current_speed(self):
        """
        Get the current speed of the stepper motor
        
        Returns:
            float: Current speed in steps per second
        """
        return self.current_speed
        
    def is_running(self):
        """
        Check if the stepper motor is currently running
        
        Returns:
            bool: True if motor is running, False otherwise
        """
        return self.running
        
    def stop(self):
        """Stop the stepper motor immediately"""
        self.running = False
        self.current_speed = 0
        
    def update(self):
        """
        Update the stepper motor state. This should be called regularly in the main loop.
        """
        if not self.running:
            return
            
        # Calculate direction
        direction = 1 if self.target_position > self.current_position else -1
        
        # Calculate distance to target
        distance = abs(self.target_position - self.current_position)
        
        # Calculate required speed based on distance and acceleration
        if distance > 0:
            # Calculate time needed to reach target
            time_to_target = math.sqrt(2 * distance / self.acceleration)
            
            # Calculate required speed
            required_speed = min(self.max_speed, 
                               self.current_speed + self.acceleration * 0.001)  # 0.001 is time step
            
            # Apply speed
            self.current_speed = required_speed
            
            # Calculate step interval (adjusted for microstepping)
            self.step_interval = 1.0 / (self.current_speed * self.microsteps)
            
            # Check if it's time to step
            current_time = time.time()
            if current_time - self.last_step_time >= self.step_interval:
                try:
                    # Step the motor with proper timing for NEMA 23
                    GPIO.output(self.dir_pin, GPIO.HIGH if direction > 0 else GPIO.LOW)
                    time.sleep(0.0001)  # Direction setup time
                    
                    # Generate step pulse
                    GPIO.output(self.step_pin, GPIO.HIGH)
                    time.sleep(0.0002)  # Pulse width for NEMA 23
                    GPIO.output(self.step_pin, GPIO.LOW)
                    time.sleep(0.0002)  # Minimum time between pulses
                    
                    self.current_position += direction
                    self.last_step_time = current_time
                    
                    # Check if we've reached the target
                    if self.current_position == self.target_position:
                        self.running = False
                        self.current_speed = 0
                except Exception as e:
                    print(f"Error during motor step: {e}")
                    self.running = False
                    self.current_speed = 0
                    
    def move_to(self, position):
        """
        Move to a specific position with current speed and acceleration settings
        
        Args:
            position (int): Target position in steps
        """
        self.set_target_position(position)
        
    def move(self, distance):
        """
        Move relative to current position
        
        Args:
            distance (int): Distance to move in steps (positive or negative)
        """
        self.set_target_position(self.current_position + distance)
        
    def run(self):
        """
        Run the stepper motor until it reaches its target position
        
        Returns:
            bool: True if motor is still running, False if target reached
        """
        self.update()
        return self.running 