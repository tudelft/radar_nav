#pragma once
#include <vector>
#include <map>
#include <functional>
#include "serial.hpp"

class Payload {
    std::vector<unsigned char> payload;
    int i = 0;

public:
    Payload() {}
    Payload(std::vector<unsigned char> payload) : payload(payload) {}

    int size() const {
        return payload.size();
    }

    uint8_t get_u8() {
        if (i >= payload.size()) return 0;
        return payload[i++];
    }

    uint16_t get_u16() {
        uint16_t res = get_u8();
        res |= get_u8() << 8;
        return res;
    }

    uint32_t get_u32() {
        uint32_t res = get_u16();
        res |= get_u16() << 16;
        return res;
    }

    void put_u8(uint8_t x) {
        payload.push_back(x);
    }

    void put_u16(uint16_t x) {
        put_u8(x & 0xff);
        put_u8((x >> 8) & 0xff);
    }

    void put_u32(uint32_t x) {
        put_u16(x & 0xffff);
        put_u16((x >> 16) & 0xffff);
    }

    const std::vector<unsigned char> &get_payload() const {
        return payload;
    }
};

template<class T, int N>
class CircBuff {
    T buffer[N];
    int buff_start = 0;
    int buff_end   = 0;
public:
    bool empty() {
        return buff_start == buff_end;
    }

    int size() {
        if (buff_end < buff_start)
            return N - buff_start + buff_end;
        else 
            return buff_end - buff_start;
    }

    void push(T x) {
        if (size() + 1 == N)
            buff_start = (buff_start + 1) % N;

        buffer[buff_end] = x;
        buff_end = (buff_end + 1) % N;
    }

    T get(int i) {
        return buffer[(buff_start + i) % N];
    }


    void pop(int n) {
        if (n > size()) buff_start = buff_end;
        buff_start = (buff_start + n) % N;
    }

    void pop() {
        pop(1);
    }

    T front() {
        return buffer[buff_start];
    }
};

class MSP {
    Serial device;
    CircBuff<unsigned char, 1024> buffer;
    std::map<unsigned char, std::function<void(Payload)>>
        callback_table;

    bool try_parse() {
        // We start with resyncing
        while (!buffer.empty() && buffer.front() != '$')
            buffer.pop();
        // Not enough data to hold valid message
        if (buffer.size() < 5) return false;

        // Check if preamble is still valid
        if (buffer.get(1) != 'M') return false;

        // Check if we have the whole message
        int size = buffer.get(3);
        if (buffer.size() < size + 6) return false;

        char msg_dir = buffer.get(2);
        unsigned char msg_cmd = buffer.get(4);
        unsigned char msg_crc = buffer.get(size + 5);
        std::vector<unsigned char> msg_payload(size);

        // Skip preamble and dir
        buffer.pop(3);

        unsigned char crc = 0;

        // Read size and cmd
        for (int i = 0; i < 2; i++) {
            crc = crc ^ buffer.front();
            buffer.pop();
        }

        // Read payload
        for (int i = 0; i < size; i++) {
            crc = crc ^ buffer.front();
            msg_payload[i] = buffer.front();
            buffer.pop();
        }

        // Check crc
        if (crc != buffer.front()) return false;
        buffer.pop();

        if (callback_table.find(msg_cmd) != callback_table.end())
            callback_table[msg_cmd](Payload(msg_payload));
        
        return true;
    }

    bool process_byte(char c) {
        buffer.push(c);
        return try_parse();
    }

    bool get_msg() {
        while (device.available()) {
            if (process_byte(device.read())) 
                return true;
        }
        return false;
    }

public:
    // MSP message identifiers
    enum : unsigned char {
        IDENT = 100,
        STATUS = 101,
        RAW_IMU = 102,
        SERVO = 103,
        MOTOR = 104,
        RC = 105,
        RAW_GPS = 106,
        COMP_GPS = 107,
        ATTITUDE = 108,
        ALTITUDE = 109,
        ANALOG = 110,
        RC_TUNING = 111,
        PID = 112,
        BOX = 113,
        MISC = 114,
        MOTOR_PINS = 115,
        BOXNAMES = 116,
        PIDNAMES = 117,
        WP = 118,
        BOXIDS = 119,
        RC_RAW_IMU = 121,
        BATTERY_STATE = 130,
        SET_RAW_RC = 200,
        SET_RAW_GPS = 201,
        SET_PID = 202,
        SET_BOX = 203,
        SET_RC_TUNING = 204,
        ACC_CALIBRATION = 205,
        MAG_CALIBRATION = 206,
        SET_MISC = 207,
        RESET_CONF = 208,
        SET_WP = 209,
        SWITCH_RC_SERIAL = 210,
        IS_SERIAL = 211,
        DEBUG = 254,
        VTX_CONFIG = 88,
        VTX_SET_CONFIG = 89,
        EEPROM_WRITE = 250,
        REBOOT = 68
    };

    MSP() : device("/dev/ttyUSB0", 115200) {}

    void send_msg(unsigned char cmd, const Payload &payload) {
        device.write('$');
        device.write('M');
        device.write('<');
        device.write((unsigned char) payload.size());
        device.write(cmd);

        unsigned char crc = 0;
        crc ^= (unsigned char) payload.size();
        crc ^= cmd;

        for (auto c : payload.get_payload()) {
            device.write(c);
            crc ^= c;
        }

        device.write(crc);
    }

    void recv_msgs() {
        while (get_msg());
    }

    void register_callback(unsigned char cmd, std::function<void(Payload)> cb) {
        callback_table[cmd] = cb;
    }
};
