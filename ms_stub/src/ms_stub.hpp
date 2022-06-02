#pragma once

#include "create_data.hpp"
#include "tools.hpp"
#include "serial.hpp"

#include <thread>
#include <atomic>
#include <cstring>

#include "timer_lib.hpp"

class MSStubSingle{
public:
    MSStubSingle(std::string &dev_path){
        createUpperBodyMsData(&data.eeprom[0x0000]);
        createUpperBodySendNoData(&data.eeprom[0x0100]);
        serial.open(dev_path);

        updata_thread = std::thread(&MSStubSingle::update, this);
        cmd_thread = std::thread( &MSStubSingle::exec, this);
    }

    ~MSStubSingle(){
        serial.close();
        shutdown.store(true);
        if (cmd_thread.joinable()) {
            cmd_thread.join();
        }

        if (updata_thread.joinable()) {
            updata_thread.join();
        }
    }

    std::vector<int> getMsIds(){
        return msids;
    }

    void addMsId(int id){
        msids.push_back(id);
    }

    void update(){
        double period_sec = 0.005;//[s]
        long period_ns = period_sec * NSEC_PER_SEC;
        timespec t1_prev;

        auto tm = getTime();
        t1_prev = tm;

        while (!shutdown.load()) {
            getNextTime(tm, period_ns);
            sleepUntil(tm);

            auto t1 = getTime();
            double cycle_sec = period_sec;//getTimeDiff_usec(t1,t1_prev)/1000000.0;

            data.mtx.lock();
            static constexpr int16_t max_ang = 13602; //最大値
            static constexpr int16_t min_ang = -13602; //最大値
            for (int idx = 0; idx < AXIS_MAX; ++idx) {

                //速度が入っている場合は、とりあえず動く
                double pos_next = data.joint[idx].pos_cur + data.joint[idx].vel_cur * cycle_sec;

                //時間指定による目標位置までの移動の場合
                if (!std::isnan(data.joint[idx].pos_tgt)) {
                    if ((data.joint[idx].pos_cur < data.joint[idx].pos_tgt && data.joint[idx].pos_tgt <= pos_next)
                     || (pos_next <= data.joint[idx].pos_tgt && data.joint[idx].pos_tgt < data.joint[idx].pos_cur)) {
                        data.joint[idx].vel_cur = 0;
                        data.joint[idx].pos_tgt = std::numeric_limits<double>::quiet_NaN();
                    }
                }

                //エンコーダの限界値でオーバーフロー
                if(pos_next > max_ang){
                    pos_next += (min_ang - max_ang);
                }else if(pos_next < min_ang){
                    pos_next += (max_ang - min_ang);
                }

                data.joint[idx].pos_cur = pos_next;
                data.joint[idx].pos = data.joint[idx].pos_cur;
            }
            data.mtx.unlock();

            t1_prev = t1;
        }
    }

    void exec() {
        timespec t1_prev;
        auto tm = getTime();
        t1_prev = tm;

        data.status.can1.temperature = true;
        data.status.can1.motor_stat = true;
        data.status.can2.motor_stat = true;

        while (!shutdown.load()) {
            if (!serial.read(&recvd, sizeof(recvd))) {
                continue;
            }

            data.mtx.lock();

            if (recvd.cmd == 0x51) {    //ファームウェアバージョン取得
                createFirmwareVersion(&recvd, &sendd);
                serial.write(&sendd, sendd.getTotalLen());
            } else if (recvd.cmd == 0x41) {    //位置取得
                createGetposResp(&data, &recvd, &sendd);
                serial.write(&sendd, sendd.getTotalLen());
            } else if (recvd.cmd == 0x14) {            //位置司令
                createMovePosResp(&data, &recvd, &sendd);
                serial.write(&sendd, sendd.getTotalLen());
            } else if (recvd.cmd == 0x15) {            //速度司令
                createMoveVelResp(&data, &recvd, &sendd);
                serial.write(&sendd, sendd.getTotalLen());
            } else if (recvd.cmd == 0x21) {            //サーボON
                execServoCmd(&data, &recvd);
                ROS_INFO("[%s] servo",serial.getport().c_str());
            } else if (recvd.cmd == 0x22) {            //スクリプト実行
                ROS_INFO("[%s] script",serial.getport().c_str());
            } else if (recvd.cmd == 0x43) {            //電圧取得
                createGetvolResp(&data, &recvd, &sendd);
                serial.write(&sendd, sendd.getTotalLen());
            } else if (recvd.cmd == 0xa0 || recvd.cmd == 0xa1) {            //cosmo
                ROS_INFO("[%s] cosmo recv cmd:0x%02x data: %s",serial.getport().c_str(), recvd.cmd, recvd.data);
            } else if (recvd.cmd == 0x00) {            //EPRAM読み込み
                createGetEEPROMResp(&data, &recvd, &sendd);
                serial.write(&sendd, sendd.getTotalLen());
            } else if (recvd.cmd == 0xFF) {            //EPRAM書き込み
                createSetEEPROMResp(&data, &recvd, &sendd);
                serial.write(&sendd, sendd.getTotalLen());
            } else if (recvd.cmd == 0x01) {            //電流値
                uint16_t current = (recvd.data[0] << 8) | recvd.data[1];            //リトルエンディアンに変換
                ROS_INFO("[%s] id:[%02d] cmd:[set current] param -> max:[%d]",serial.getport().c_str(), recvd.msid, current);
            } else if (recvd.cmd == 0x22) {            //スクリプト実行
                uint16_t script_no = (recvd.data[0] << 8) | recvd.data[1];            //リトルエンディアンに変換
                ROS_INFO("[%s] id:[%02d] cmd:[run script] param -> no:[%d]",serial.getport().c_str(), recvd.msid, script_no);
            }
            data.mtx.unlock();
        }
    }

    template<class T>
    void sendOtherCmd(T* cmd){
        serial.write(cmd,cmd->getTotalLen());
    }


private:
    std::thread cmd_thread;
    std::thread updata_thread;

    std::vector<int> msids; // 本USBに紐づくMSのリスト

    std::atomic<bool> shutdown = false;
    MsRecvRaw recvd = { 0 };
    AeroSendRaw sendd = { 0 };
    MSData data = { 0 };

    Serial serial;
};


