#pragma once

#include <hardware_interface/other_command_interface_helper.hpp>
#include <sensor_msgs/Joy.h>
#include "cosmo_joy_data.hpp"

namespace cosmo_joy_controller {

class CosmoJoyController: public controller_interface::OtherCommandInterfaceHelper {
public:
    CosmoJoyController();

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void execute(const BuffRaw *buf_recv, BuffRaw *buf_send) override;

    std::vector<std::pair<uint8_t, uint8_t>> handleHeaders() override {
        return { {0xfb,0xbf}, {0xbf,0xfb}};
    }

    void setJoy(const CosmoJoyCmdReqType &data, sensor_msgs::Joy &joy_) {
        joy_.buttons[0] = data.buttons.btnX; //□ = X
        joy_.buttons[1] = data.buttons.btnY; //△ = Y
        joy_.buttons[2] = data.buttons.btnA; //× = A
        joy_.buttons[3] = data.buttons.btnB; //○ = B

        joy_.buttons[4] = data.buttons.btnLB; //L1 = LB
        joy_.buttons[5] = data.buttons.btnRB; //R1 = RB
        joy_.buttons[6] = data.buttons.btnLT; //L2 = LT
        joy_.buttons[7] = data.buttons.btnRT; //R2 = RT

        joy_.buttons[8] = data.buttons.btnLP; //Analog L push
        joy_.buttons[9] = data.buttons.btnRP; //Analog R push
        joy_.buttons[10] = data.buttons.btnSL; //select
        joy_.buttons[11] = data.buttons.btnST; //start

        joy_.axes[0] = data.L_LR;  //left joystick
        joy_.axes[1] = data.L_UD;  //left joystick
        joy_.axes[2] = data.R_UD;  //right joystick
        joy_.axes[3] = data.R_LR;  //right joystick

             if (data.buttons.btnLA)joy_.axes[4] =  1;
        else if (data.buttons.btnRA)joy_.axes[4] = -1;
        else                        joy_.axes[4] =  0;

             if (data.buttons.btnUA)joy_.axes[5] =  1;
        else if (data.buttons.btnDA)joy_.axes[5] = -1;
        else                        joy_.axes[5] =  0;

        joy_.header.stamp = ros::Time::now();
        joy_.header.frame_id = "/cosmo/joy";
    }

private:
    ros::Publisher joy_pub;
    sensor_msgs::Joy joy_msg;
};

}
