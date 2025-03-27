# TTROBOT_SERVO

This repository contains the SCServo Linux library (version 220329) for controlling FEETECH servo motors.

## Contents

The repository includes:

- SCServo Linux library
- Example applications for SMS/STS and SCSCL series servos
- TTRobot control examples

## Features

- Support for multiple servo series (SMS, STS, SCSCL)
- Various control modes (position, speed, etc.)
- Synchronous and asynchronous control options
- Communication via serial interface

## Usage

### Building the Library

```bash
cd SCServo_Linux
cmake .
make
```

### Running Examples

Navigate to any example directory and build:

```bash
cd examples/SMS_STS/Ping
cmake .
make
sudo ./Ping /dev/ttyACM0
```

## License

This code is distributed as provided by FEETECH.

## Notes

This is a mirror of the SCServo_Linux_220329 library for use with the TTROBOT project. 
