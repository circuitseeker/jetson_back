#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time

# Define the GPIO pin (adjust the number according to your wiring)
LED_PIN = 12

# Use BOARD or BCM numbering (ensure this matches your pin selection)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED_PIN, GPIO.OUT)

try:
    while True:
        GPIO.output(LED_PIN, GPIO.HIGH)  # Turn LED on
        time.sleep(1)                    # Wait for 1 second
        GPIO.output(LED_PIN, GPIO.LOW)   # Turn LED off
        time.sleep(1)                    # Wait for 1 second
except KeyboardInterrupt:
    GPIO.cleanup()  # Clean up GPIO settings on exit
