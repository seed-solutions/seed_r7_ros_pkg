#include <ros/ros.h>
#include <cstring>
#include "create_data.hpp"
#include "tools.hpp"

void createMovePosResp(MSData *data, MsRecvRaw *recvd,AeroSendRaw *sendd){
    int msno = 0;
    int data_len = 0;

    //受信処理
    if (!data->status.can1.motor_stat && !data->status.can2.motor_stat) {
        data_len = recvd->ad - 2;            //msidとcmdを除く

        if (recvd->msid == 0) {
            uint16_t tgt_time = ((recvd->data[60] << 8) + recvd->data[61]);
            for (msno = 0; msno < (data_len - 2) / 2; msno++) {
                if (!(recvd->data[msno * 2] == 0x7f && recvd->data[msno * 2 + 1] == 0xff)) {
                    if (tgt_time == 0) {
                        tgt_time = 2;
                    }
                    double tgt_time_sec = tgt_time/100.;
                    int16_t tgt_pos = (recvd->data[msno * 2] << 8) + recvd->data[msno * 2 + 1];
                    data->joint[msno].pos_tgt = tgt_pos;
                    data->joint[msno].vel_cur = (data->joint[msno].pos_tgt - data->joint[msno].pos_cur) / tgt_time_sec;
                }
            }
        } else if (recvd->msid - 1 < AXIS_MAX) {
            msno = recvd->msid - 1;
            if (!(recvd->data[msno * 2] == 0x7f && recvd->data[msno * 2 + 1] == 0xff)) {
                uint16_t tgt_time = 2;
                double tgt_time_sec = tgt_time/100.;
                int16_t tgt_pos = (recvd->data[msno * 2] << 8) + recvd->data[msno * 2 + 1];
                data->joint[msno].pos_tgt = tgt_pos;
                data->joint[msno].vel_cur = (data->joint[msno].pos_tgt - data->joint[msno].pos_cur) / tgt_time_sec;
            }
        }
    }else{
        ROS_ERROR("[ms-stub] cannot move because of servo-OFF state.");
    }

    //送信処理
    sendd->init();
    sendd->header[0] = 0xDF;
    sendd->header[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->ad = 64;
    sendd->msid = recvd->msid;
    if (recvd->msid == 0) {
        for (msno = 0; msno < 30; msno++) {
            sendd->data[msno * 2 + 0] = ((uint8_t*) &data->joint[msno].pos)[1];
            sendd->data[msno * 2 + 1] = ((uint8_t*) &data->joint[msno].pos)[0];
        }
        sendd->data[60] = *(uint8_t*)&data->status.can2;
        sendd->data[61] = *(uint8_t*)&data->status.can1;
    } else {
        msno = recvd->msid - 1;
        sendd->data[msno * 2 + 0] = ((uint8_t*) &data->joint[msno].pos)[1];
        sendd->data[msno * 2 + 1] = ((uint8_t*) &data->joint[msno].pos)[0];
    }

    sendd->addChecksum();
}

void createMoveVelResp(MSData *data, MsRecvRaw *recvd,AeroSendRaw *sendd){
    int msno = 0;
    int data_len = 0;

    //受信処理
    if (!data->status.can1.motor_stat && !data->status.can2.motor_stat) {

        data_len = recvd->ad - 2;            //msidとcmdを除く
        if (recvd->msid == 0) {
            for (msno = 0; msno < (data_len - 2) / 2; msno++) {
                if (!(recvd->data[msno * 2] == 0x7f && recvd->data[msno * 2 + 1] == 0xff)) {
                    int16_t vel = (recvd->data[msno * 2] << 8) + recvd->data[msno * 2 + 1];
                    data->joint[msno].vel_cur = vel;
                }
            }
        } else if (recvd->msid - 1 < AXIS_MAX) {
            msno = recvd->msid - 1;
            if (!(recvd->data[msno * 2] == 0x7f && recvd->data[msno * 2 + 1] == 0xff)) {
                int16_t vel = (recvd->data[msno * 2] << 8) + recvd->data[msno * 2 + 1];
                data->joint[msno].vel_cur = vel;
            }
        }
    }else{
        ROS_ERROR("[ms-stub] cannot move because of servo-OFF state.");
    }

    //送信処理
    sendd->init();
    sendd->header[0] = 0xDF;
    sendd->header[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->ad = 64;
    sendd->msid = recvd->msid;
    if (recvd->msid == 0) {
        for (msno = 0; msno < 30; msno++) {
            sendd->data[msno * 2 + 0] = ((uint8_t*) &data->joint[msno].pos)[1];
            sendd->data[msno * 2 + 1] = ((uint8_t*) &data->joint[msno].pos)[0];
        }
        sendd->data[60] = *(uint8_t*)&data->status.can2;
        sendd->data[61] = *(uint8_t*)&data->status.can1;
    } else {
        msno = recvd->msid - 1;
        sendd->data[msno * 2 + 0] = ((uint8_t*) &data->joint[msno].pos)[1];
        sendd->data[msno * 2 + 1] = ((uint8_t*) &data->joint[msno].pos)[0];
    }

    sendd->addChecksum();
}

void execServoCmd(MSData *data, MsRecvRaw *recvd){
    data->status.can1.motor_stat = false;
    data->status.can2.motor_stat = false;
}

void createSetEEPROMResp(MSData *data, MsRecvRaw *recvd,AeroSendRaw *sendd){
    int ofst = 1;//アドレスが1バイトを使うため
    uint8_t datalen = 0;
//            int ofst = 2;//アドレスが1バイトを使うため

    uint8_t *tmp = &recvd->msid;
    uint16_t address = (tmp[0]<<8)|tmp[1];//リトルエンディアンに変換

    sendd->init();
    sendd->header[0] = 0xCF;
    sendd->header[1] = 0xFC;

    //Aerocheckerによると、送信データ+2バイトのデータが帰ってくる？？？
    sendd->ad = recvd->ad + 2;

    sendd->cmd = recvd->cmd;
    sendd->msid = recvd->msid;
    memcpy(sendd->data,recvd->data,sizeof(recvd->data));

    datalen = recvd->ad - 3;

    dump("eeprom write",address,recvd->ad,datalen,recvd->data,ofst);

    memcpy(&data->eeprom[address],&recvd->data[ofst],recvd->ad-3);//アドレスとコマンドの3バイトを除いてコピー

}

void createFirmwareVersion(MsRecvRaw *recvd,AeroSendRaw *sendd){
    sendd->init();
    sendd->header[0] = 0xDF;
    sendd->header[1] = 0xFD;
    sendd->ad = 7;
    sendd->cmd = recvd->cmd;
    sendd->msid = 0x00;
    sendd->data[0] = 0x01;
    sendd->data[1] = 0x23;
    sendd->data[2] = 0x45;
    sendd->data[3] = 0x67;
    sendd->data[4] = 0x89;
    sendd->addChecksum();
}

void createGetposResp(MSData *data, MsRecvRaw *recvd, AeroSendRaw *sendd) {
    int msno = 0;
    sendd->init();
    sendd->header[0] = 0xDF;
    sendd->header[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->msid = recvd->msid;
    if (recvd->msid == 0) {
        sendd->ad = 64;
        for (msno = 0; msno < 30; msno++) {
            sendd->data[msno * 2 + 0] = ((uint8_t*) &data->joint[msno].pos)[1];
            sendd->data[msno * 2 + 1] = ((uint8_t*) &data->joint[msno].pos)[0];
        }
        sendd->data[60] = *(uint8_t*)&data->status.can2;
        sendd->data[61] = *(uint8_t*)&data->status.can1;
    } else {
        sendd->ad = 4;
        msno = recvd->msid - 1;
        sendd->data[msno * 2 + 0] = ((uint8_t*) &data->joint[msno].pos)[1];
        sendd->data[msno * 2 + 1] = ((uint8_t*) &data->joint[msno].pos)[0];
    }

    sendd->addChecksum();
}

void createGetvolResp(MSData *data, MsRecvRaw *recvd, AeroSendRaw *sendd) {
    int data_len = 0;
    int msno = 0;
    sendd->init();
    sendd->header[0] = 0xDF;
    sendd->header[1] = 0xFD;
    sendd->cmd = recvd->cmd;
    sendd->ad = 64;
    sendd->msid = 0x00;

    data_len = sendd->ad - 2;
    for (msno = 0; msno < (data_len - 2) / 2; msno++) {
        sendd->data[msno * 2 + 0] = data->joint[msno].tmp;
        sendd->data[msno * 2 + 1] = data->joint[msno].vol;
    }
    sendd->data[60] = 0xFF;
    sendd->data[61] = 0x00;

    sendd->addChecksum();
}

void createGetEEPROMResp(MSData *data, MsRecvRaw *recvd, AeroSendRaw *sendd) {
    //TODO アドレスで処理分け

    int ofst = 1;//アドレスが1バイトを使うため
    uint8_t *tmp = &recvd->msid;
    uint16_t address = (tmp[0]<<8)|tmp[1];//リトルエンディアンに変換
    uint8_t data_len = recvd->data[1]; //32

    //data
    sendd->init();

    //header
    sendd->header[0] = 0xCF;
    sendd->header[1] = 0xFC;
    sendd->ad = data_len + 3;//データ長+コマンド1バイト+アドレス2バイト(35)
    sendd->cmd = recvd->cmd;

    //address
    sendd->msid = recvd->msid;
    sendd->data[0] = recvd->data[0];
    memcpy(&sendd->data[ofst],&data->eeprom[address],data_len);

    dump("eeprom read",address,sendd->ad,data_len,sendd->data,ofst);

}

void createUpperBodyMsData(uint8_t* data){
    int ofst = 1;
    data[0+ofst] = 1; // msid
    data[1+ofst] = 0xFF; // smsid
    data[2+ofst] = 1; // 機体設定(0:None 1:Noid 2:Lifter 3:Mover)
    data[3+ofst] = 0; // Protocol(0:Aero3 1:Aero4)
    data[4+ofst] = 1; // Use Cloud
}

void createUpperBodySendNoData(uint8_t* data){
    int ofst = 1;
    //neck
    data[0+ofst] = 10;
    data[1+ofst] = 12;
    data[2+ofst] = 11;

    //right
    data[3+ofst] = 1;
    data[4+ofst] = 2;
    data[5+ofst] = 3;
    data[6+ofst] = 4;
    data[7+ofst] = 5;
    data[8+ofst] = 6;
    data[9+ofst] = 7;

    //waist
    data[10+ofst] = 9;

    //reserved
    data[11+ofst] = 0;
    data[12+ofst] = 0;
    data[13+ofst] = 0;
    data[14+ofst] = 0;

    //waist
    data[15+ofst] = 25;

    //reserved
    data[16+ofst] = 0;
    data[17+ofst] = 0;

    //left
    data[18+ofst] = 16;
    data[19+ofst] = 17;
    data[20+ofst] = 18;
    data[21+ofst] = 19;
    data[22+ofst] = 20;
    data[23+ofst] = 21;
    data[24+ofst] = 22;

    //waist
    data[25+ofst] = 24;

    //reserved
    data[26+ofst] = 0;
    data[27+ofst] = 0;
    data[28+ofst] = 0;
    data[29+ofst] = 0;
    data[30+ofst] = 0;
}
