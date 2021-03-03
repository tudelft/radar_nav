#include "serial.hpp"
#include <stdexcept>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

Serial::Serial(const std::string &port, int baud_rate) {
    fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        throw std::runtime_error("Error opening serial device");
    }
    int baud = B9600;

    switch (baud_rate) {
        case 4800:   baud = B4800;   break;
        case 9600:   baud = B9600;   break;
        case 19200:  baud = B19200;  break;
        case 38400:  baud = B38400;  break;
        case 115200: baud = B115200; break;
    }

    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        throw std::runtime_error("Error getting termios options of device");
    }
    
    cfsetospeed(&options, (speed_t) baud);
    cfsetispeed(&options, (speed_t) baud);

    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        throw std::runtime_error("Error setting termios options of device");
    }
}

Serial::~Serial() {
    close(fd);
}

bool Serial::available() {
    int n;
    ioctl(fd, FIONREAD, &n);
    if (n < 0) {
        throw std::runtime_error("Error checking amount of data in serial device");
    }
    return n > 0;
}

unsigned char Serial::read() {
    unsigned char c;
    int s = ::read(fd, &c, 1);
    if (s != 1) {
        throw std::runtime_error("Error reading from serial device");
    }
    return c;
}

void Serial::write(unsigned char c) {
    int s = ::write(fd, &c, 1);
    if (s != 1) {
        throw std::runtime_error("Error writing to serial device");
    }
}
