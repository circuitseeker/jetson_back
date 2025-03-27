/*
* ServoScanner Tool
* Scans for servos with IDs 1-7 and displays their current positions
* Uses SyncRead to efficiently read positions from multiple servos
* Outputs positions as angles between 0-360 degrees
*/

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include "SCServo.h"

SMS_STS sm_st;

// Function to convert raw position to angle (0-360 degrees)
float convertToAngle(int16_t position) {
    // Assuming position range is 0-4095 (12-bit)
    return (position / 4095.0f) * 360.0f;
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
    
    // Scan for connected servos
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
    
    // Get current position of all connected servos
    uint8_t* IDs = connectedIDs.data();
    uint8_t numServos = connectedIDs.size();
    uint8_t rxPacket[2]; // Only need 2 bytes for position
    int16_t position;
    float angle;
    
    // Initialize SyncRead
    sm_st.syncReadBegin(numServos, sizeof(rxPacket));
    
    // Display header
    std::cout << "\n------ Current Servo Positions ------" << std::endl;
    std::cout << "ID\tRaw Position\tAngle (degrees)" << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    
    // Perform SyncRead to get positions
    sm_st.syncReadPacketTx(IDs, numServos, SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket));
    
    for(uint8_t i=0; i < numServos; i++) {
        if(!sm_st.syncReadPacketRx(IDs[i], rxPacket)) {
            std::cout << "ID:" << (int)IDs[i] << " sync read error!" << std::endl;
            continue;
        }
        
        position = sm_st.syncReadRxPacketToWrod(15); // Read position with direction bit
        angle = convertToAngle(position);
        
        std::cout << (int)IDs[i] << "\t" << position << "\t\t" 
                  << std::fixed << std::setprecision(2) << angle << "°" << std::endl;
    }
    
    // Check for mirrored servo pairs
    std::cout << "\n------ Mirrored Servo Pairs ------" << std::endl;
    
    // Check if we have both servos in a pair
    bool hasPair1_2 = (std::find(connectedIDs.begin(), connectedIDs.end(), 1) != connectedIDs.end() &&
                      std::find(connectedIDs.begin(), connectedIDs.end(), 2) != connectedIDs.end());
    
    bool hasPair3_4 = (std::find(connectedIDs.begin(), connectedIDs.end(), 3) != connectedIDs.end() &&
                      std::find(connectedIDs.begin(), connectedIDs.end(), 4) != connectedIDs.end());
    
    bool hasPair5_6 = (std::find(connectedIDs.begin(), connectedIDs.end(), 5) != connectedIDs.end() &&
                      std::find(connectedIDs.begin(), connectedIDs.end(), 6) != connectedIDs.end());
    
    if(hasPair1_2) std::cout << "Pair 1-2: FOUND (Mirrored)" << std::endl;
    else std::cout << "Pair 1-2: INCOMPLETE" << std::endl;
    
    if(hasPair3_4) std::cout << "Pair 3-4: FOUND (Mirrored)" << std::endl;
    else std::cout << "Pair 3-4: INCOMPLETE" << std::endl;
    
    if(hasPair5_6) std::cout << "Pair 5-6: FOUND (Mirrored)" << std::endl;
    else std::cout << "Pair 5-6: INCOMPLETE" << std::endl;
    
    if(std::find(connectedIDs.begin(), connectedIDs.end(), 7) != connectedIDs.end())
        std::cout << "Servo 7: FOUND (Individual)" << std::endl;
    else 
        std::cout << "Servo 7: NOT FOUND" << std::endl;
    
    // Clean up
    sm_st.syncReadEnd();
    sm_st.end();
    std::cout << "\nScan complete." << std::endl;
    
    return 0;
} 