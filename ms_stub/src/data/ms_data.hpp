#pragma once

#include <stdint.h>
#include <mutex>

#define AXIS_MAX 100

struct SingleCANErrorInfo {
    bool connection  :1;
    bool calibration :1;
    bool motor_stat  :1;
    bool temperature :1;
    bool response    :1;
    bool step_out    :1;
    bool protective_stopped :1;
    bool power       :1;
};


struct MSErrorInfo {
    SingleCANErrorInfo can2;
    SingleCANErrorInfo can1;
};


typedef struct {
    double pos_cur;
    double pos_tgt = std::numeric_limits<double>::quiet_NaN();
    double vel_cur = 0;

    int16_t pos;
    uint8_t vol;
    uint8_t tmp;
} MSJointData;

typedef struct {
    MSErrorInfo status;
    MSJointData joint[AXIS_MAX];
    uint8_t eeprom[65535];
    std::mutex mtx;
} MSData;

