#pragma once

#include <ros/ros.h>
#include "ms_driver.hpp"
#include "aero3_command.hpp"
#include "other_cmd_buff.hpp"
#include "robot_status.hpp"

class RobotDriverSingleUSB {
public:
    RobotDriverSingleUSB(std::string usb_port,int baudrate):usb_port(usb_port) {
        aero_driver = new AeroCommand();

        if(aero_driver->openPort(usb_port, baudrate)){
            ROS_INFO_STREAM("usb port was successfully opened. :" <<usb_port);
        }else{
            ROS_ERROR_STREAM("usb port was not opened. :" <<usb_port);
        }
    }

    void addMsDriver(MSDriver* ms_driver){
        ms_drivers.push_back(ms_driver);
    }

    ~RobotDriverSingleUSB() {
        delete aero_driver;
    }

    int getNumMs(){
        return ms_drivers.size();
    }

    int getMsId(int idx){
        return ms_drivers[idx]->getMsId();
    }

    int getNumJoints(){
        int num = 0;
        for(auto &ms_driver:ms_drivers){
            num += ms_driver->getNumJoints();
        }
        return num;
    }

    //非RT
    std::string getJointName(int idx) {
        int ofst = 0;
        size_t msidx = 0;
        for (msidx = 0; msidx < ms_drivers.size(); ++msidx) {
            auto ms_driver = ms_drivers[msidx];
            if ((idx - ofst) >= ms_driver->getNumJoints()) {
                ofst += ms_driver->getNumJoints();
            } else {
                break;
            }
        }
        if (msidx < ms_drivers.size()) {
            auto name = ms_drivers[msidx]->getJointName(idx - ofst);
            return name;
        }
        return "";
    }

    //非RT
    std::vector<std::string> getJointNames(){
        std::vector < std::string > ret;
        for(auto &ms_driver:ms_drivers){
            auto jnames = ms_driver->getJointNames();
            ret.insert(ret.end(), jnames.begin(), jnames.end());
        }
        return ret;
    }

    //非RT
    std::string getPortName(){
        return usb_port;
    }

    void read() {
        aero_driver->read();
    }


    void getPosition(int16_t* positions){
        int ofst = 0;
        for(auto &ms_driver:ms_drivers){
            ms_driver->getPosition(&positions[ofst],aero_driver);
            ofst += ms_driver->getNumJoints();
        }
    }

    int getMsStatus(MSStatus *ms_status, int ofst) {
        for (size_t msidx = 0; msidx < ms_drivers.size(); ++msidx) {
            ms_drivers[msidx]->getMsStatus(ms_status[msidx + ofst].err_u16, aero_driver);
        }
        return ms_drivers.size();
    }

    void getUSBStatus(USBStatus& usb_status){
        usb_status.disconnect = !aero_driver->connected();
    }


    //速度は取得できない
//    void getActualVelocities(std::vector<int16_t> &velocities){
//        for(auto &ms_driver:ms_drivers){
//            ms_driver->getActualVelocities(velocities,aero_driver);
//        }
//    }

    void sendVelocity(const int16_t* vel) {
        int ofst = 0;
        for(auto &ms_driver:ms_drivers){
            ms_driver->sendVelocity(&vel[ofst],aero_driver);
            ofst += ms_driver->getNumJoints();
        }
    }

    //ROS形式の並び順でのストローク
    void sendPosition(const int16_t* pos, const double &tgt_time_sec) {
        int ofst = 0;
        for(auto &ms_driver:ms_drivers){
            ms_driver->sendPosition(&pos[ofst],tgt_time_sec,aero_driver);
            ofst += ms_driver->getNumJoints();
        }
    }

    void runScript(const uint16_t* script_no){
        int ofst = 0;
        for(auto &ms_driver:ms_drivers){
            ms_driver->runScript(&script_no[ofst],aero_driver);
            ofst += ms_driver->getNumJoints();
        }
    }

    void send(){
        for(auto &ms_driver:ms_drivers){
            ms_driver->send(aero_driver);
        }
    }


    void getOtherCommands(int msidx, BuffList &cmds) {
        ms_drivers[msidx]->getOtherCommands(cmds, aero_driver);
    }

    void sendOtherCommands(int msidx, BuffList &cmds) {
        ms_drivers[msidx]->sendOtherCommands(cmds, aero_driver);
    }
private:
    std::string usb_port;
    AeroCommand *aero_driver = nullptr;
    std::vector<MSDriver*> ms_drivers;//一つのmmsに複数のmsがつながっている可能性があるので。
};
