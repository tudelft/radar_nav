#pragma once
#include <string>

class Serial {
    int fd;
public:
    Serial(const std::string& port, int baud_rate);
    ~Serial();
    bool available();
    unsigned char read();
    void write(unsigned char);
};
