#include <pluginlib/class_list_macros.hpp>
#include <controller_manager_msgs/ListControllers.h>
#include "motion_player.hpp"

namespace motion_player {

MotionPlayer::MotionPlayer(){
}

bool MotionPlayer::init(hardware_interface::OtherCommandInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {

    std::vector<std::string> ms_ids_str;

    ms_ids_str = hw->getNames();
    for (auto msid_str : ms_ids_str) {
        handles.push_back(hw->getHandle(msid_str));
        ms_ids.push_back(std::stoi(msid_str));
    }
    req_server.init(controller_nh,&swp_buff_req,&swp_buff_resp, &stop_request);

    return true;
}


MotionPlayer::~MotionPlayer(){
}

void MotionPlayer::starting(const ros::Time &time) {
}

void MotionPlayer::update(const ros::Time &time, const ros::Duration &period) {

    if(!curdata){
        curdata = swp_buff_req.readFromB();
        if(curdata){
            step = curdata->step;
        }
    }else{
        ++step;
    }


    bool running = false;
    if(curdata){
        running = true;
    }

    if(stop_request.load()){
        curdata = nullptr;
        stop_request.store(false);

        //TODO 停止軌道
    }

    while (curdata) {
        if (step == curdata->step) {
            int msid = curdata->msid;
            for(size_t idx = 0;idx<ms_ids.size();++idx){
                if(msid == ms_ids[idx]){
                    handles[msid].setSendCmd(&curdata->rawdata);
                }
            }
            curdata = swp_buff_req.readFromB(true);
        }else{
            break;
        }
    }

    if(running){
        Response resp = {step,!curdata};
        swp_buff_resp.writeFromA(resp);
    }

}

void MotionPlayer::stopping(const ros::Time &time) {
}


}

PLUGINLIB_EXPORT_CLASS(motion_player::MotionPlayer, controller_interface::ControllerBase)
