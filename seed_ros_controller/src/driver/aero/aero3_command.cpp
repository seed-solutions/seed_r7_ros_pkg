#include "aero3_command.hpp"
#include <iostream>

#include "debug_tools.hpp"

///////////////////////////////
AeroCommand::AeroCommand() {
}

///////////////////////////////
AeroCommand::~AeroCommand() {
}

bool AeroCommand::openPort(std::string _port, unsigned int _baud_rate) {
    return serial_com.open(_port, _baud_rate);
}

void AeroCommand::closePort() {
    serial_com.close();
}

bool AeroCommand::connected() {
    return serial_com.connected();
}

void AeroCommand::read() {
    int read_size = 1;
    int other_cmd_recvd_no = 0;
    other_cmd_recvd.size = 0;
    while (read_size != 0) {
        uint8_t recv_data[256] = {0};
#if 1 // 頭出し付きの受信
        //受信データの頭出し作業
        bool found = false;
        size_t rsize = 0;
        size_t rsize_tot = 0;
        read_size = 0;
        while (1) {
            recv_data[0] = recv_data[1];
            rsize = serial_com.read(&recv_data[1], 1);
            if (rsize == 0) {
                break;
            }

            if (recv_data[0] == 0x00 || recv_data[1] == 0x00) {
                continue;
            }

            //ヘッダ判定
            if ((((recv_data[0] & 0x0F) << 4) == (recv_data[1] & 0xF0)) && ((recv_data[0] & 0xF0) == ((recv_data[1] & 0x0F) << 4))) {
                found = true;
                break;
            }
        }

        //ヘッダらしきデータがない場合は受信不可
        if(!found){
            break;
        }

        rsize_tot += 2;
        size_t expect_size = 0;
        if (recv_data[0] == 0xfe || recv_data[0] == 0xef){
            expect_size = 64;
        }
        else if (recv_data[0] == 0xbf || recv_data[0] == 0xfb){
            expect_size = 11;
        }
        else {
            //データ長を受信
            rsize = serial_com.read(&recv_data[2], 1);
            if (rsize < 1) {
                break;
            }
            rsize_tot += rsize;
            expect_size = recv_data[2] + sizeof(Header) + 2;//チェックサム、ヘッダ、lenはlenのサイズに含まれないので、足しておく
        }

        //期待サイズになるまで受信データを展開
        uint8_t *head = &recv_data[rsize_tot];
        while(rsize_tot < expect_size){
            rsize = serial_com.read(head, expect_size - rsize_tot);
            if(rsize == 0){
                break;
            }
            rsize_tot += rsize;
            head += rsize;
        }

        //期待したデータ量が得られなかった場合は、受信失敗
        if(rsize_tot != expect_size){
            break;
        }

        read_size = rsize_tot;

#else //単純な受信
        read_size = serial_com.read(recv_data, sizeof(recv_data));
        if (read_size == 0) {
            break;
        }
#endif
        Header *header = reinterpret_cast<Header*>(recv_data);
        if (header->data[0] == 0xDF && header->data[1] == 0xFD) {
            //aero
            RecvBuffAero *aero_cmd = reinterpret_cast<RecvBuffAero*>(recv_data);
            if (!aero_cmd->isvalid()) {
                printf("invalid packet received\n");
                continue;
            }

            int msid = 0; //msidはとりあえず0固定
            MSState *tgt_state = &cur_state[msid];

            if (aero_cmd->cmd == 0x14 || aero_cmd->cmd == 0x15 || aero_cmd->cmd == 0x41) {
                if (aero_cmd->mcid == 0x00) {
                    //制御命令応答(位置)
                    int jidx = 0;
                    for (int idx = 0; idx + 1 < aero_cmd->datalen() - 2; idx += 2) {
                        aero_cmd->get(idx, tgt_state->mcval[jidx++].pos);
                    }
                    aero_cmd->get(aero_cmd->datalen() - 2, tgt_state->status);
                }
            } else if (aero_cmd->cmd == 0x42) {
                //電流指令値取得返信
                int jidx = 0;
                for (int idx = 0; idx + 1 < aero_cmd->datalen() - 2; idx += 2) {
                    aero_cmd->get(idx, tgt_state->mcval[jidx++].current);
                }
            } else if (aero_cmd->cmd == 0x43) {
                //温度・電圧取得返信
                int jidx = 0;
                for (int idx = 0; idx + 1 < aero_cmd->datalen() - 2; idx += 2) {
                    aero_cmd->get(idx, tgt_state->mcval[jidx].voltage);
                    aero_cmd->get(idx + 1, tgt_state->mcval[jidx].temp);
                    ++jidx;
                }
            } else if (aero_cmd->cmd == 0x00) {
                //EEPROMデータ読み込み返信

            } else if (aero_cmd->cmd == 0xFF) {
                //EEPROMデータ書き込み返信

            }

        } else {
            //その他コマンド
            memcpy(other_cmd_recvd.buf[other_cmd_recvd_no].data, recv_data, read_size);
            other_cmd_recvd.buf[other_cmd_recvd_no].size = read_size;
            other_cmd_recvd.size = other_cmd_recvd_no + 1;
            other_cmd_recvd_no++;
            //キャパシティを超えそうな場合は、次回にまわす。
            if(other_cmd_recvd.cmd_capacity() == other_cmd_recvd_no){
                break;
            }
        }

//        dump("read", recv_data, read_size);
    }
}

void AeroCommand::sendPGET(int msid) {
    buff.init();
    buff.header[0] = 0xFD;
    buff.header[1] = 0xDF;
    buff.cmd = 0x41;
    buff.mcid = 0x00; //全データ取得

    buff.addChecksum();

    serial_com.write(buff.serialize(), buff.getTotalLen());
}

void AeroCommand::sendMOVE(int msid, const double &tgt_time_sec, int16_t *data) {
    buff.init();
    buff.header[0] = 0xFD;
    buff.header[1] = 0xDF;
    buff.cmd = 0x14;
    buff.mcid = 0x00;

    int next_idx = 0;
    uint16_t tgt_time = static_cast<uint16_t>(tgt_time_sec * 100);
    next_idx = buff.setData(data, 30, next_idx);
    next_idx = buff.setData(tgt_time, next_idx);

    buff.addChecksum();

    serial_com.write(buff.serialize(), buff.getTotalLen());
}

void AeroCommand::sendTURN(int msid, int16_t *data) {
    buff.init();
    buff.header[0] = 0xFD;
    buff.header[1] = 0xDF;
    buff.cmd = 0x15;
    buff.mcid = 0x00;

    int next_idx = 0;
    next_idx = buff.setData(data, 30, next_idx);
    next_idx = buff.setData(uint16_t(0x7FFF), next_idx);
    buff.addChecksum();

    serial_com.write(buff.serialize(), buff.getTotalLen());
}

void AeroCommand::sendSCRIPT(int msid, uint16_t wait_time, uint16_t *data) {
    buff.init();
    buff.header[0] = 0xFD;
    buff.header[1] = 0xDF;
    buff.cmd = 0x22;
    buff.mcid = 0x00; //全MCに送信

    int next_idx = 0;
    next_idx = buff.setData(data, 30, next_idx);
    next_idx = buff.setData(wait_time, next_idx);
    buff.addChecksum();

    serial_com.write(buff.serialize(), buff.getTotalLen());
}

int16_t AeroCommand::getpos(int msid, int joint) {
    return cur_state[0].mcval[joint].pos;
}

uint16_t AeroCommand::getstatus(int msid) {
    return cur_state[0].status;
}

void AeroCommand::getOtherCommands(int msid, BuffList &cmds) {
    memcpy(&cmds, &this->other_cmd_recvd, sizeof(this->other_cmd_recvd));
}

void AeroCommand::sendOtherCommands(int msid, BuffList &cmds) {
    for (int idx = 0; idx < cmds.size; ++idx) {
        serial_com.write(cmds.buf[idx].data, cmds.buf[idx].size);
        cmds.buf[idx].size = 0;
    }
    cmds.size = 0;
}

