/*
* TTRobot Servo Controller
* Controls 7 servos with the following configuration:
* - Servo 1 and 2 are mirrored (2 gets 360-angle of 1)
* - Servo 3 and 4 are mirrored (4 gets 360-angle of 3)
* - Servo 5 and 6 are mirrored (6 gets 360-angle of 5)
* - Servo 7 is an individual servo
*
* Input format from terminal: Angle1,Angle3,Angle5,Angle7
* Special commands:
* - SET: Calibrates all connected servos to set current position as the middle position (180 degrees)
* - q/quit/exit: Exits the program
*/

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include "SCServo.h"

SMS_STS sm_st;

// Function to convert angle (0-360 degrees) to raw position (0-4095)
int16_t convertToPosition(float angle) {
    // Clamp angle to 0-360 range
    angle = std::max(0.0f, std::min(360.0f, angle));
    // Map 0-360 degrees to 0-4095
    return static_cast<int16_t>(round((angle / 360.0f) * 4095.0f));
}

// Function to convert raw position to angle (0-360 degrees)
float convertToAngle(int16_t position) {
    // Assuming position range is 0-4095 (12-bit)
    return (position / 4095.0f) * 360.0f;
}

// Function to calibrate a servo setting current position as middle (180 degrees)
void calibrateServo(uint8_t id) {
    if (sm_st.CalibrationOfs(id) == 1) {
        std::cout << "Servo " << static_cast<int>(id) << " calibrated successfully" << std::endl;
    } else {
        std::cout << "Failed to calibrate servo " << static_cast<int>(id) << std::endl;
    }
}

// Function to parse the input string into angles
bool parseInput(const std::string& input, float& angle1, float& angle3, float& angle5, float& angle7) {
    std::stringstream ss(input);
    std::string item;
    std::vector<float> angles;
    
    // Parse comma-separated values
    while (std::getline(ss, item, ',')) {
        try {
            float angle = std::stof(item);
            angles.push_back(angle);
        } catch (const std::exception& e) {
            std::cerr << "Error parsing input: " << e.what() << std::endl;
            return false;
        }
    }
    
    // Check if we have exactly 4 values
    if (angles.size() != 4) {
        std::cerr << "Error: Expected 4 angles (format: Angle1,Angle3,Angle5,Angle7)" << std::endl;
        return false;
    }
    
    angle1 = angles[0];
    angle3 = angles[1];
    angle5 = angles[2];
    angle7 = angles[3];
    
    return true;
}

int main(int argc, char **argv)
{
    if(argc<2){
        std::cout << "argc error!" << std::endl;
        std::cout << "Usage: " << argv[0] << " <serial_port>" << std::endl;
        return 0;
    }
    
    std::cout << "Serial port: " << argv[1] << std::endl;
    if(!sm_st.begin(1000000, argv[1])){
        std::cout << "Failed to init sms/sts motor!" << std::endl;
        return 0;
    }
    
    // Check which servos are connected
    std::vector<uint8_t> connectedIDs;
    std::cout << "Scanning for servos with IDs 1-7..." << std::endl;
    
    for(int id = 1; id <= 7; id++) {
        int result = sm_st.Ping(id);
        if(result != -1) {
            std::cout << "Found servo ID: " << id << std::endl;
            connectedIDs.push_back(id);
        }
    }
    
    if(connectedIDs.empty()) {
        std::cout << "No servos found! Check connections and try again." << std::endl;
        sm_st.end();
        return 0;
    }
    
    // Main control loop
    std::string input;
    float angle1, angle3, angle5, angle7;
    float angle2, angle4, angle6;
    
    std::cout << "\n=========================================" << std::endl;
    std::cout << "TTRobot Servo Controller" << std::endl;
    std::cout << "Enter angles in format: Angle1,Angle3,Angle5,Angle7" << std::endl;
    std::cout << "Special commands:" << std::endl;
    std::cout << "  SET - Calibrate all servos to set current position as middle (180°)" << std::endl;
    std::cout << "  q/quit/exit - Exit the program" << std::endl;
    std::cout << "=========================================" << std::endl;
    
    while(true) {
        std::cout << "> ";
        std::getline(std::cin, input);
        
        if(input == "q" || input == "quit" || input == "exit") {
            break;
        }
        
        // Special command: SET - Calibrate all servos
        if(input == "SET") {
            std::cout << "\nCalibrating all connected servos..." << std::endl;
            std::cout << "This will set the current position of each servo as the middle position (180°)" << std::endl;
            
            for(uint8_t id : connectedIDs) {
                calibrateServo(id);
            }
            
            std::cout << "Calibration complete. All servos now have their current positions set as 180°" << std::endl;
            continue;
        }
        
        if(!parseInput(input, angle1, angle3, angle5, angle7)) {
            std::cout << "Please enter angles in format: Angle1,Angle3,Angle5,Angle7" << std::endl;
            continue;
        }
        
        // Calculate mirrored angles for servos 2, 4, and 6
        angle2 = 360.0f - angle1;
        angle4 = 360.0f - angle3;
        angle6 = 360.0f - angle5;
        
        // Display the angles being applied
        std::cout << "\nApplying angles:" << std::endl;
        std::cout << "Servo 1: " << angle1 << "° | Servo 2: " << angle2 << "° (mirrored)" << std::endl;
        std::cout << "Servo 3: " << angle3 << "° | Servo 4: " << angle4 << "° (mirrored)" << std::endl;
        std::cout << "Servo 5: " << angle5 << "° | Servo 6: " << angle6 << "° (mirrored)" << std::endl;
        std::cout << "Servo 7: " << angle7 << "°" << std::endl;
        
        // Convert angles to positions
        int16_t pos1 = convertToPosition(angle1);
        int16_t pos2 = convertToPosition(angle2);
        int16_t pos3 = convertToPosition(angle3);
        int16_t pos4 = convertToPosition(angle4);
        int16_t pos5 = convertToPosition(angle5);
        int16_t pos6 = convertToPosition(angle6);
        int16_t pos7 = convertToPosition(angle7);
        
        // Define servo speeds and acceleration
        const uint16_t speed = 1500;
        const uint8_t acc = 50;
        
        // Move servos to the specified positions
        if(std::find(connectedIDs.begin(), connectedIDs.end(), 1) != connectedIDs.end()) {
            sm_st.WritePosEx(1, pos1, speed, acc);
        }
        if(std::find(connectedIDs.begin(), connectedIDs.end(), 2) != connectedIDs.end()) {
            sm_st.WritePosEx(2, pos2, speed, acc);
        }
        if(std::find(connectedIDs.begin(), connectedIDs.end(), 3) != connectedIDs.end()) {
            sm_st.WritePosEx(3, pos3, speed, acc);
        }
        if(std::find(connectedIDs.begin(), connectedIDs.end(), 4) != connectedIDs.end()) {
            sm_st.WritePosEx(4, pos4, speed, acc);
        }
        if(std::find(connectedIDs.begin(), connectedIDs.end(), 5) != connectedIDs.end()) {
            sm_st.WritePosEx(5, pos5, speed, acc);
        }
        if(std::find(connectedIDs.begin(), connectedIDs.end(), 6) != connectedIDs.end()) {
            sm_st.WritePosEx(6, pos6, speed, acc);
        }
        if(std::find(connectedIDs.begin(), connectedIDs.end(), 7) != connectedIDs.end()) {
            sm_st.WritePosEx(7, pos7, speed, acc);
        }
        
        // Wait for movement to complete (approximation)
        usleep(500 * 1000); // 500ms
        
        std::cout << "Movement complete." << std::endl;
    }
    
    std::cout << "Exiting TTRobot Servo Controller." << std::endl;
    sm_st.end();
    return 0;
}

