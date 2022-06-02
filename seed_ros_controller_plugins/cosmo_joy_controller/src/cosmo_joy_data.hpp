#pragma once
#include <string>
#include <cstring>
#include <sstream>

#include <hardware_interface/other_cmd_buff.hpp>

namespace cosmo_joy_controller {

struct JoyButtons {
    bool btnLP :1; //Analog L push
    bool btnSL :1; //select
    bool btnRP :1; //Analog R push
    bool btnST :1; //start
    bool btnUA :1; //↑
    bool btnRA :1; //→
    bool btnDA :1; //↓
    bool btnLA :1; //←

    bool btnLT :1; //L2 = LT
    bool btnRT :1; //R2 = RT
    bool btnLB :1; //L1 = LB
    bool btnRB :1; //R1 = RB
    bool btnY :1; //△ = Y
    bool btnB :1; //○ = B
    bool btnA :1; //× = A
    bool btnX :1; //□ = X
};

struct CosmoJoyCmdReqType {
    JoyButtons buttons;
    double L_LR;
    double L_UD;
    double R_LR;
    double R_UD;


    void fromRawBuffer(const BuffRaw &buff) {
        struct MsRecvRaw {
            uint8_t header[2];
            uint8_t address;
            uint8_t cmd;
            uint8_t data[251];

        };
        const MsRecvRaw *recvd = reinterpret_cast<const MsRecvRaw*>(buff.data);
        memcpy(&buttons, recvd->data, sizeof(JoyButtons));
        L_LR = (recvd->data[2] - 127.0) / 127.0;
        L_UD = (127.0 - recvd->data[3]) / 127.0;
        R_UD = (127.0 - recvd->data[5]) / 127.0;
        R_LR = (recvd->data[4] - 127.0) / 127.0;
    }
};
}
