#pragma once

#include <hardware_interface/other_command_interface.hpp>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/moving_time_interface.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <libs/swap_buffer.hpp>
#include <thread>


#include "motion_request_server.hpp"
#include "structure_motion.hpp"
#include "rt_bridge.hpp"

namespace motion_player {

class MotionPlayer: public controller_interface::Controller<hardware_interface::OtherCommandInterface>
{
public:
    MotionPlayer();

    ~MotionPlayer();

private:
    bool init(hardware_interface::OtherCommandInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
    void starting(const ros::Time &time) override;
    void update(const ros::Time &time, const ros::Duration &period) override;
    void stopping(const ros::Time &time) override;

private:
    ros::Timer timer;

    std::vector<int> ms_ids;
    std::vector<hardware_interface::OtherCommandHandle> handles;

    MotionRequestServer req_server;

    reqBuffType swp_buff_req;
    respBuffType swp_buff_resp;
    Request* curdata = nullptr;

    std::atomic<bool> stop_request = false;
    int step = 0;
};
}
