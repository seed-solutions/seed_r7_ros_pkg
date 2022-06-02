#pragma once

#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include "ms_stub.hpp"

class Cosmo{
public:
    struct UsbInfo{
        MSStubSingle* usb = nullptr;
    };

    Cosmo(){

    }

    ~Cosmo(){
        is_shutdown.store(true);
        if(th.joinable()){
            th.join();
        }
    }

    void addUsb(MSStubSingle *usbDriver){
        usb_infos.push_back(usbDriver);
    }

    void run(){
        th = std::thread(&Cosmo::cosmoCmdThread,this);
    }

    std::vector<std::string> split(std::string str,std::string delim){
        std::vector<std::string> result;
        size_t pos = 0;
        while ((pos = str.find(delim)) != std::string::npos) {
            result.push_back(str.substr(0, pos));
            str.erase(0, pos + delim.length());
        }
        result.push_back(str);

        return result;
    }

    void send(int msid,std::string cosmo_cmd){
        ROS_INFO_STREAM("cosmo start -> msid: "<<msid<<" cmd: "<<cosmo_cmd);
        MSStubSingle* tgt_driver = nullptr;
        for(auto &usb_info:usb_infos){
            for(int info_msid:usb_info->getMsIds()){
                if(msid == info_msid){
                    tgt_driver = usb_info;
                    break;
                }
            }

            if(tgt_driver){
                break;
            }
        }
        if (tgt_driver) {
            CosmoSendRaw data;
            data.setHeader(msid);
            data.setData(cosmo_cmd);
            data.addChecksum();
            tgt_driver->sendOtherCmd(&data);
        }
    }

    void cosmoCmdThread(){

        fd_set fds, readfds;
        int fd_in = fileno(stdin);
        int maxfd = fd_in;

        FD_ZERO(&readfds);
        FD_SET(fd_in, &readfds);

        //タイムアウト
        struct timeval tv_to;

        std::cout<<"\033[32m"<<"input cosmo request [msid,command] >> "<< "\033[m";
        fflush(stdout);

        while(!is_shutdown.load()){
            std::string input_str;
            char buff[256];

            memcpy(&fds, &readfds, sizeof(fd_set));
            tv_to.tv_sec = 1;
            tv_to.tv_usec = 0;
            select(maxfd+1, &fds, NULL, NULL, &tv_to);

            if (FD_ISSET(fd_in, &fds)) {
                std::cin.getline(buff, 256);
                input_str = std::string(buff);
                if(input_str.empty()){
                    std::cout<<"\033[32m"<<"input cosmo request [msid,command] >> "<< "\033[m";
                    fflush(stdout);
                    continue;
                }
            } else {
                continue;
            }

            std::vector<std::string> words = split(input_str,",");
            if(words.size() !=2){
                ROS_WARN("invalud request.");
                continue;
            }

            int msid = std::stoi(words[0]);
            char cosmo_cmd[100] = { 0 };
            memcpy(cosmo_cmd,words[1].c_str(),words[1].length());

            send(msid,cosmo_cmd);

            std::cout<<"\033[32m"<<"input cosmo request [msid,command] >> "<< "\033[m";
            fflush(stdout);
        }
    }

private:
    std::atomic<bool> is_shutdown = false;
    std::thread th;
    std::vector<MSStubSingle*> usb_infos;

};
