#ifndef SCSERVO_H
#define SCSERVO_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

class SMS_STS {
public:
    SMS_STS();
    ~SMS_STS();
    
    bool begin(int baud, const char* port);
    void end();
    
    int WritePosEx(int id, int16_t pos, uint16_t speed, uint8_t acc);
    int ReadPos(int id);
    
private:
    int fd;
    bool is_connected;
    
    int write_data(const uint8_t* data, size_t length);
    int read_data(uint8_t* data, size_t length);
};

#endif // SCSERVO_H 