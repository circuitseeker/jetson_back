#!/usr/bin/env python3
import ctypes
import os
import serial

# Load the compiled library
lib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'TTROBOT_SERVO/SCServo_Linux/build/libSCServo.so')
lib = ctypes.CDLL(lib_path)

# Define function signatures for SMS_STS class methods
lib._ZN7SMS_STSC1Ev.argtypes = []
lib._ZN7SMS_STSC1Ev.restype = None

lib._ZN7SMS_STS10WritePosExEhsth.argtypes = [ctypes.c_void_p, ctypes.c_ubyte, ctypes.c_short, ctypes.c_ushort, ctypes.c_ubyte]
lib._ZN7SMS_STS10WritePosExEhsth.restype = ctypes.c_int

lib._ZN7SMS_STS7ReadPosEi.argtypes = [ctypes.c_void_p, ctypes.c_int]
lib._ZN7SMS_STS7ReadPosEi.restype = ctypes.c_int

class SCServo:
    def __init__(self, port):
        self.port = port
        self.serial = None
        
    def open(self):
        """Open the serial port"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=1000000,  # 1Mbps
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            return True
        except Exception as e:
            print(f"Error opening serial port: {e}")
            return False
        
    def close(self):
        """Close the serial port"""
        if self.serial:
            self.serial.close()
            self.serial = None
            
    def write_pos(self, id, pos):
        """Write position to servo"""
        if not self.serial:
            return False
            
        # Convert angle to position value (4095 steps = 360 degrees)
        pos_value = int((pos * 4095) / 360)
        
        # SMS_STS protocol:
        # Header: 0xFF 0xFF
        # ID: servo id
        # Length: number of parameters + 3
        # Instruction: 0x03 (write)
        # Parameter 1: register address (0x2A for goal position)
        # Parameter 2: position low byte
        # Parameter 3: position high byte
        # Checksum: ~(ID + Length + Instruction + P1 + P2 + P3) & 0xFF
        
        data = bytearray([
            0xFF, 0xFF,  # Header
            id,  # ID
            5,  # Length
            0x03,  # Write instruction
            0x2A,  # Goal position register
            pos_value & 0xFF,  # Position low byte
            (pos_value >> 8) & 0xFF,  # Position high byte
        ])
        
        # Calculate checksum
        checksum = 0
        for b in data[2:]:  # Skip header
            checksum += b
        checksum = (~checksum) & 0xFF
        data.append(checksum)
        
        try:
            self.serial.write(data)
            return True
        except Exception as e:
            print(f"Error writing to servo: {e}")
            return False
        
    def read_pos(self, id):
        """Read current position of servo"""
        if not self.serial:
            return -1
            
        # SMS_STS protocol:
        # Header: 0xFF 0xFF
        # ID: servo id
        # Length: 4
        # Instruction: 0x02 (read)
        # Parameter 1: register address (0x38 for present position)
        # Parameter 2: number of bytes to read (2)
        # Checksum: ~(ID + Length + Instruction + P1 + P2) & 0xFF
        
        data = bytearray([
            0xFF, 0xFF,  # Header
            id,  # ID
            4,  # Length
            0x02,  # Read instruction
            0x38,  # Present position register
            2,  # Number of bytes to read
        ])
        
        # Calculate checksum
        checksum = 0
        for b in data[2:]:  # Skip header
            checksum += b
        checksum = (~checksum) & 0xFF
        data.append(checksum)
        
        try:
            self.serial.write(data)
            
            # Read response
            # Response format:
            # Header: 0xFF 0xFF
            # ID: servo id
            # Length: 4
            # Error: error code
            # Parameter 1: position low byte
            # Parameter 2: position high byte
            # Checksum
            
            # Read until we get the header
            while True:
                if self.serial.read(1) == b'\xff' and self.serial.read(1) == b'\xff':
                    break
                    
            resp_id = ord(self.serial.read(1))
            length = ord(self.serial.read(1))
            error = ord(self.serial.read(1))
            
            if error != 0:
                print(f"Servo error: {error}")
                return -1
                
            pos_low = ord(self.serial.read(1))
            pos_high = ord(self.serial.read(1))
            checksum = ord(self.serial.read(1))
            
            # Verify checksum
            calc_checksum = (~(resp_id + length + error + pos_low + pos_high)) & 0xFF
            if calc_checksum != checksum:
                print("Checksum error")
                return -1
                
            # Convert position value to angle (4095 steps = 360 degrees)
            pos = (pos_high << 8) | pos_low
            return (pos * 360) / 4095
            
        except Exception as e:
            print(f"Error reading from servo: {e}")
            return -1 