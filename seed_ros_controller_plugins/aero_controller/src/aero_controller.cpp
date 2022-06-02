#include <pluginlib/class_list_macros.hpp>
#include "aero_controller.hpp"
#include "aero_data.hpp"

namespace aero_controller {

AeroController::AeroController(){
};

bool AeroController::init(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){
    svon_srv = controller_nh.advertiseService("servo_on", &AeroController::servoOn, this);
    return true;
}

bool AeroController::servoOn(aero_controller::ServoOn::Request &req, aero_controller::ServoOn::Response &res) {

    AeroDataReqType reqdata;
    reqdata.init();
    reqdata.header[0] = 0xFD;
    reqdata.header[1] = 0xDF;
    reqdata.cmd = 0x21;
    reqdata.mcid = 0x00; //全MCに送信

    int next_idx = 0;
    uint16_t on = 0x0001;
    uint16_t nop = 0x7FFF;
    next_idx = reqdata.setData(on, 30, next_idx);
    next_idx = reqdata.setData(nop, next_idx);
    reqdata.addChecksum();

    BuffRaw buff;
    memcpy(buff.data,reqdata.serialize(),reqdata.getTotalLen());
    buff.size = reqdata.getTotalLen();

    auto mslist = getMsList();
    for (int &ms : mslist) {
        addSendData(ms, buff);
    }
    return true;
}

}

PLUGINLIB_EXPORT_CLASS(aero_controller::AeroController, controller_interface::ControllerBase)
