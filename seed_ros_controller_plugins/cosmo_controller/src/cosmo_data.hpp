#pragma once
#include <string>
#include <cstring>
#include <sstream>

#include <hardware_interface/other_cmd_buff.hpp>

namespace cosmo_controller {
struct CosmoCmdReqType {
    int header_type;
    uint8_t addr;
    uint8_t cmd_type;
    uint8_t msid;
    std::string cmd_str;

    void fromRawBuffer(const BuffRaw &buff) {

        struct MsRecvRaw {
            uint8_t header[2];
            uint8_t address;
            uint8_t cmd;
            uint8_t msid;
            uint8_t data[251];

        };

        const MsRecvRaw *recvd = reinterpret_cast<const MsRecvRaw*>(buff.data);

        //headerの受信
        if (recvd->header[0] == 0xef) {
            this->header_type = 0;
        } else if (recvd->header[0] == 0xfe) { //EF FE形式で受け取ったら
            this->header_type = 1;
        }

        this->cmd_type = recvd->cmd;

        //コマンド内容の受信
        int idx = 0;
        std::ostringstream convert;
        while (recvd->data[idx] != 0x00) {
            convert << (char) recvd->data[idx++];
        }
        std::string cmd_str = std::string(convert.str());

        this->addr = recvd->address;
        this->msid = recvd->msid;
        this->cmd_str = cmd_str;
    }
};

struct CosmoCmdRespType {
    int header_type;
    uint8_t addr;
    uint8_t cmd_type;
    uint8_t msid;
    std::string cmd_str;

    void toRawBuffer(BuffRaw &buff) {
        buff.size = 64;
        if (header_type == 0) {
            buff.data[0] = 0xef;
            buff.data[1] = 0xfe;
        } else if (header_type == 1) {
            buff.data[0] = 0xfe;
            buff.data[1] = 0xef;
        }
        buff.data[2] = addr;
        buff.data[3] = cmd_type;
        buff.data[4] = msid;

        strcpy((char*) &buff.data[5], cmd_str.c_str());

        buff.addChecksum();
    }
};

}
