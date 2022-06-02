#pragma once

#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include "ms_stub.hpp"

class CosmoJoy{
public:
    CosmoJoy(ros::NodeHandle &nh,MSStubSingle* tgt_ms):tgt_ms(tgt_ms){
        joy_sub = nh.subscribe("joy", 2, &CosmoJoy::receiveJoyTopic, this);
    }
private:
    void receiveJoyTopic(const sensor_msgs::JoyPtr &joy_data) {
        //ここで、usb経由でコマンドを投げる
        CosmoJoySendRaw data;
        data.setHeader(1);

        joy_data->axes;
        if(joy_data->buttons.size() > 0 )data.data[1] |= (joy_data->buttons[0]  & 0x01) << 7;
        if(joy_data->buttons.size() > 1 )data.data[1] |= (joy_data->buttons[1]  & 0x01) << 4;
        if(joy_data->buttons.size() > 2 )data.data[1] |= (joy_data->buttons[2]  & 0x01) << 6;
        if(joy_data->buttons.size() > 3 )data.data[1] |= (joy_data->buttons[3]  & 0x01) << 5;
        if(joy_data->buttons.size() > 4 )data.data[1] |= (joy_data->buttons[4]  & 0x01) << 2;
        if(joy_data->buttons.size() > 5 )data.data[1] |= (joy_data->buttons[5]  & 0x01) << 3;
        if(joy_data->buttons.size() > 6 )data.data[1] |= (joy_data->buttons[6]  & 0x01) << 0;
        if(joy_data->buttons.size() > 7 )data.data[1] |= (joy_data->buttons[7]  & 0x01) << 1;
        if(joy_data->buttons.size() > 8 )data.data[0] |= (joy_data->buttons[8]  & 0x01) << 0;
        if(joy_data->buttons.size() > 9 )data.data[0] |= (joy_data->buttons[9]  & 0x01) << 2;
        if(joy_data->buttons.size() > 10)data.data[0] |= (joy_data->buttons[10] & 0x01) << 1;
        if(joy_data->buttons.size() > 11)data.data[0] |= (joy_data->buttons[11] & 0x01) << 3;


        if(joy_data->axes.size() > 0)data.data[2] = (joy_data->axes[0] + 1) * 127;
        if(joy_data->axes.size() > 1)data.data[3] = (1 - joy_data->axes[1]) * 127;
        if(joy_data->axes.size() > 2)data.data[4] = (1 - joy_data->axes[2]) * 127;
        if(joy_data->axes.size() > 3)data.data[5] = (joy_data->axes[3] + 1) * 127;
        if(joy_data->axes.size() > 4)if(joy_data->axes[4] ==  1)data.data[0] |= 0x01 << 7; // ←
        if(joy_data->axes.size() > 4)if(joy_data->axes[4] == -1)data.data[0] |= 0x01 << 5; // →
        if(joy_data->axes.size() > 5)if(joy_data->axes[5] ==  1)data.data[0] |= 0x01 << 4; // ↑
        if(joy_data->axes.size() > 5)if(joy_data->axes[5] == -1)data.data[0] |= 0x01 << 6; // ↓

        data.addChecksum();

//        dump("cosmo_joy",0x00,data.getTotalLen(),sizeof(data),(uint8_t*)&data,0);
        tgt_ms->sendOtherCmd(&data);
    }

private:
    ros::Subscriber joy_sub;
    MSStubSingle* tgt_ms = nullptr;
};
