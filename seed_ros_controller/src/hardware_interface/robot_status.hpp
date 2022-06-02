#pragma once

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

struct MSStatus {
    int msid;
    uint16_t err_u16;
    void clear(){
        err_u16 = 0x0000;
    }
};

struct USBStatus {
    uint8_t disconnect;
    void clear(){
        disconnect = 0x00;
    }

};

struct Status {
    static constexpr int ms_num_max = 256; //!< MSの数の最大値
    int ms_num = 0;
    MSStatus ms_status[ms_num_max] = { 0 };
    int get_ms_capacity(){return ms_num_max;}

    static constexpr int usb_num_max = 256;//!< USBの数の最大値
    int usb_num = 0;
    USBStatus usb_status[usb_num_max] = { 0 };
    int get_usb_capacity(){return usb_num_max;}

};

