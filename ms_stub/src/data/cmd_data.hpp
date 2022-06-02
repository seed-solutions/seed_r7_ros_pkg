#pragma once

#include <string>

struct  CosmoSendRaw{
    uint8_t header[2];
    uint8_t ad;
    uint8_t cmd;
    uint8_t msid;
    uint8_t data[59];

    void setHeader(uint8_t msid){
        this->header[0] = 0xEF;
        this->header[1] = 0xFE;
        this->ad = 0;    //送信元
        this->cmd = 0xa1;
        this->msid = msid;
    }

    void setData(std::string d){
        for (unsigned int idx = 0; idx < sizeof(data); ++idx) {
            data[idx] = 0xFF;
        }
        int cmd_len = d.length()+1;
        memcpy(this->data, d.c_str(), cmd_len);
        addChecksum();
    }

    void addChecksum(){
        unsigned int cs = 0;

        cs += ad;
        cs += cmd;
        cs += msid;
        for (size_t idx = 0; idx < sizeof(data)-1; idx++){
            cs += data[idx];
        }

        data[sizeof(data)-1] = static_cast<uint8_t>(~cs);
    }

    int getTotalLen() const{
        return 64;
    }
};


struct  CosmoJoySendRaw{
    uint8_t header[2];
    uint8_t length;
    uint8_t msid;
    uint8_t data[7];


    void setHeader(uint8_t msid) {
        for (unsigned int idx = 0; idx < sizeof(data); ++idx) {
            data[idx] = 0x00;
        }
        this->header[0] = 0xFB;
        this->header[1] = 0xBF;
        this->msid = msid;
        this->length = sizeof(data) + 2;
    }

    void addChecksum(){
        unsigned int cs = 0;

        cs += msid;
        for (size_t idx = 0; idx < sizeof(data)-1; idx++){
            cs += data[idx];
        }

        data[sizeof(data)-1] = static_cast<uint8_t>(~cs);
    }

    int getTotalLen() const{
        return 11;
    }
};

struct  AeroSendRaw{
    uint8_t header[2];
    uint8_t ad;
    uint8_t cmd;
    uint8_t msid;
    uint8_t data[251];


    void init() {
        for (unsigned int idx = 0; idx < sizeof(data); ++idx) {
            data[idx] = 0xFF;
        }
    }

    void addChecksum(){
        unsigned int cs = 0;

        cs += ad;
//        cs += cmd;
        cs += msid;
        for (int idx = 0; idx < ad - 2; idx++){
            cs += data[idx];
        }

        data[ad - 2] = static_cast<uint8_t>(~cs);
    }

    int getTotalLen() const{
        return ad + 4; // header*2 + len + checksum
    }
};

struct MsRecvRaw{
    uint8_t header[2];
    uint8_t ad;
    uint8_t cmd;
    uint8_t msid;
    uint8_t data[251];

} ;
