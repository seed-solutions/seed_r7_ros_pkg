#pragma once

#include "other_command_interface.hpp"
#include <controller_interface/controller.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <libs/swap_buffer.hpp>

#include <condition_variable>
#include <mutex>
#include <thread>

namespace controller_interface {
class OtherCommandInterfaceHelper: public controller_interface::Controller<hardware_interface::OtherCommandInterface> {
public:
    OtherCommandInterfaceHelper() {
        thread_nrt = std::thread(&OtherCommandInterfaceHelper::nrt_thread, this);
    }

    virtual ~OtherCommandInterfaceHelper(){
        running.store(false);
        mtx.lock();
        cond.notify_one();
        mtx.unlock();
        if(thread_nrt.joinable()){
            thread_nrt.join();
        }
    }

    virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        return false;
    }

    virtual void execute(const BuffRaw* buf_recv,BuffRaw* buf_send){
    }

    virtual std::vector<std::pair<uint8_t,uint8_t>> handleHeaders(){
        return std::vector<std::pair<uint8_t,uint8_t>>();
    }

    bool addSendData(int msid, const BuffRaw &buf_send) {
        int idx = -1;
        auto itr = std::find(port_names.begin(),port_names.end(),std::to_string(msid));
        if(itr != port_names.end()){
            idx = std::distance(port_names.begin(), itr);
            swp_buff_send[idx].writeFromA(buf_send);
            return true;
        }

        return false;
    }

    std::vector<int> getMsList(){
        std::vector<int> ret;
        for(auto port:port_names){
            ret.push_back(std::stoi(port));
        }
        return ret;
    }

private:

    bool init(hardware_interface::OtherCommandInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override {

        port_names = hw->getNames();
        for (auto port : port_names) {
            handles.push_back(hw->getHandle(port));
        }
        buff.resize(handles.size());

        handle_headers = handleHeaders();

        return init(root_nh,controller_nh);
    }

    void update(const ros::Time &time, const ros::Duration &period) override {
        bool cmd_exist = false;

        for (size_t idx = 0; idx < handles.size(); ++idx) {
            buff[idx].size = 0;

            //受信バッファから読み出す
            for (size_t idx2 = 0; idx2 < handle_headers.size(); ++idx2) {
                handles[idx].getCmd(handle_headers[idx2].first, handle_headers[idx2].second, &buff[idx]);
            }

            for (int idx2 = 0; idx2 < buff[idx].size; ++idx2) {
                swp_buff_recv[idx].writeFromA(buff[idx].buf[idx2]);
                cmd_exist = true;
            }

            //送信バッファを詰める
            BuffRaw *recv_buff = nullptr;
            while (1) {
                recv_buff = swp_buff_send[idx].readFromB();
                if (!recv_buff) {
                    break;
                }
                handles[idx].setSendCmd(recv_buff);
            }
        }

        //非RTスレッドを動かす
        if (cmd_exist) {
            std::unique_lock < std::mutex > lk(mtx, std::try_to_lock);
            if (lk.owns_lock()) {
                cond.notify_one();
            }
        }
    }

    void starting(const ros::Time &time) override {
    }

    void stopping(const ros::Time &time) override {
    }


    void nrt_thread() {
        while (running.load()) {
            std::unique_lock < std::mutex > lk(mtx);
            cond.wait(lk);

            for (size_t idx = 0; idx < sizeof(swp_buff_recv) / sizeof(swp_buff_recv[0]); ++idx) {
                while (1) {
                    BuffRaw data_send;
                    data_send.size = 0;
                    const BuffRaw *data_recv = swp_buff_recv[idx].readFromB();
                    if (!data_recv) {
                        break;
                    }

                    execute(data_recv,&data_send);

                    if (data_send.size != 0) {
                        swp_buff_send[idx].writeFromA(data_send);
                    }
                }
            }
        }
    }

private:
    std::atomic<bool> running = true;
    std::vector<std::string> port_names;
    std::mutex mtx;
    std::condition_variable cond;
    std::vector<std::pair<uint8_t,uint8_t>> handle_headers;

    std::thread thread_nrt;
    std::vector<hardware_interface::OtherCommandHandle> handles; //!< シリアルポートごとの、コマンドハンドラ
    std::vector<BuffList> buff;
    SwapBuffer<BuffRaw, 100> swp_buff_recv[5];                   //!< シリアルポートごとの、受信バッファ
    SwapBuffer<BuffRaw, 100> swp_buff_send[5];                   //!< シリアルポートごとの、送信バッファ
};

}

