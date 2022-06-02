#include <pluginlib/class_list_macros.hpp>
#include "cosmo_joy_controller.hpp"

namespace cosmo_joy_controller {

CosmoJoyController::CosmoJoyController(){
    joy_msg.axes.resize(6,0);
    joy_msg.buttons.resize(13,0);
}

bool CosmoJoyController::init(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){
    joy_pub = controller_nh.advertise<sensor_msgs::Joy>("joy",1);
    return true;
}

void CosmoJoyController::execute(const BuffRaw *buf_recv, BuffRaw *buf_send) {
    CosmoJoyCmdReqType req;
    req.fromRawBuffer(*buf_recv);
    setJoy(req, joy_msg);
    joy_pub.publish(joy_msg);
}

}

PLUGINLIB_EXPORT_CLASS(cosmo_joy_controller::CosmoJoyController, controller_interface::ControllerBase)
