#include <ros/ros.h>
#include "seed_r7_ros_controller/GetAeroCommand.h"

#define IREX_2022

class RobotStatusCmd {
    enum {
        SUCCESS, FAIL, INVALID
    };

public:
    RobotStatusCmd(const ros::NodeHandle &_nh, robot_hardware::RobotHW *_in_hw) :
            nh_(_nh), hw_(_in_hw) {
        //client = nh_.serviceClient<seed_r7_ros_controller::CosmoCommand>("/robot_status");
        robot_status_timer = nh_.createTimer(ros::Duration(1), &RobotStatusCmd::run, this);

        if (ros::ok()) {
            robot_status_srv = nh_.advertiseService("get_robot_status", &RobotStatusCmd::doCommand, this);
        }
    }
    ~RobotStatusCmd() {

    }

    //CosmoCMDの送受信を定義、基本的には来たら返す
    void run(const ros::TimerEvent &_event) {

        while (1) {
            auto cmd = hw_->getRobotStatusCmd();
            if (cmd.header_type < 0) {
                break;
            }
            exec_cmd(cmd);
        }
    }

    bool exec_cmd(RobotStatusCmdReqType cmd) {
        for(int idx = 0; idx < 68; idx++){
            current_data[idx] = cmd.recvd_data[idx];
        }
        return true;
    }

    bool doCommand(seed_r7_ros_controller::GetAeroCommand::Request &req, seed_r7_ros_controller::GetAeroCommand::Response &res) {
#ifndef IREX_2022
        //もしMSに問い合わせをするようになったらここに実装する
#else
        ROS_INFO_STREAM("DO COMMAND CALL hexNum"<<req.hexNum);
        //とりあえずはここに記録した値を返す
        if (req.hexNum == "0x61") {

                varData.clear();
                for(int idx = req.start; idx < req.start+req.number; idx++){
                    varData.push_back(current_data[idx]);
                }
                unsigned int allVar;
                switch(varData.size()){
                case 1:
                    allVar = varData[0];
                    break;
                case 2:
                    allVar = (varData[0] << 8) + varData[1];
                    break;
                case 3:
                    allVar = (varData[0] << 16) + (varData[1] << 8) + varData[2];
                    break;
                case 4:
                    allVar = (varData[0] << 24) + (varData[1] << 16) + (varData[2] << 8) + varData[3];
                    break;
                default:
                    break;
                }
                ROS_INFO_STREAM("robot_status 30to33 : "<<allVar);
                res.data = allVar;
        }

        res.result = "success";
        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient client;
    robot_hardware::RobotHW *hw_;
    ros::Timer robot_status_timer;
    ros::ServiceServer robot_status_srv;
    uint8_t current_data[68] = {0};
    std::vector<uint8_t> varData;
};

#endif
