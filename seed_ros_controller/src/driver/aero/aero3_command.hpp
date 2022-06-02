#pragma once

#include "serial_com.hpp"
#include "send_buff.hpp"
#include "recv_buff.hpp"
#include "other_cmd_buff.hpp"

class AeroCommand {
public:
    AeroCommand();
    ~AeroCommand();

    bool openPort(std::string _port, unsigned int _baud_rate);
    void closePort();

    void read();

    bool connected();


    void sendPGET(int msid);
    void sendMOVE(int msid, const double &tgt_time_sec, int16_t *data);
    void sendTURN(int msid, int16_t *data);
    void sendSCRIPT(int msid,uint16_t wait_time, uint16_t *data);
    void sendOtherCommands(int msid, BuffList &cmds);

    int16_t getpos(int msid,int joint);
    uint16_t getstatus(int msid);
    void getOtherCommands(int msid, BuffList &cmds);
private:
    static constexpr int ms_num_capacity = 100; //!< 通信可能なmsの数の最大値
    MSState cur_state[ms_num_capacity];//msごとに用意
    BuffList other_cmd_recvd;

    SerialCommunication serial_com;
    SendBuff buff;
};

