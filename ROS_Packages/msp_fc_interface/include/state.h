#pragma once

typedef struct __attribute__((packed)) {
    float x;
    float y;
    float z;
} vec3f_t;

typedef struct __attribute__((packed)) {
    float roll;
    float pitch;
    float yaw;
} att3f_t;

typedef struct __attribute__((packed)) {
    vec3f_t pos;
    vec3f_t vel;
    att3f_t att;
} robot_t;

/* template for holding both floats (control.cpp) and uint16_t (msp.cpp)
   control values are calculated as floats and sent over to MSP as uint16_t */
template<typename T>
struct signals {
    T xb;
    T yb;
    T zb;
    T thr;
    // initialize to zero
    signals () {
        xb = 0;
        yb = 0;
        zb = 0;
        thr = 0;
    }
};

typedef enum {
    DISARM = 0,
    ARM = 1
} arm_status_t;