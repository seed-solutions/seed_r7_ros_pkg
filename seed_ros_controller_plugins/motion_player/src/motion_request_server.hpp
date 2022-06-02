#pragma once

#include <ros/ros.h>
#include <queue>

#include "structure_motion.hpp"
#include "rt_bridge.hpp"
#include "finger_info.hpp"
#include "arm_info.hpp"

#include <pluginlib/class_loader.h>
#include <plugin_interfaces/stroke_converter/stroke_converter_base.hpp>

#include <motion_player/Load.h>
#include <motion_player/PlayAction.h>
#include <actionlib/server/simple_action_server.h>

#include "func_extension_list.hpp"

namespace motion_player {

struct JointTarget{
    std::vector<double> tgtpos;
};

class MotionRequestServer
{

    typedef actionlib::SimpleActionServer<motion_player::PlayAction> PlayActionServer;
    typedef std::shared_ptr<actionlib::SimpleActionServer<motion_player::PlayAction>> PlayActionServerPtr;

public:
    MotionRequestServer() : converter_loader_("seed_ros_controller", "seed_converter::StrokeConverter"){
        extension_list = new ExtensionList();
    }

    ~MotionRequestServer();

    void init(ros::NodeHandle& my_nh,reqBuffType *req_buff,respBuffType *resp_buff,std::atomic<bool>* stop_request);

private:
    bool load(motion_player::Load::Request &req, motion_player::Load::Response &res);

    void execute(const motion_player::PlayGoalConstPtr& goal);

    void sendThread();

private:
    ros::ServiceServer load_srv;
    PlayActionServerPtr play_as = nullptr;

    reqBuffType *req_buff = nullptr;
    respBuffType *resp_buff = nullptr;

    std::vector<std::string> joint_names;
    double control_period_sec = 20;
    FingerInfoList finger_info;
    ArmInfoList arm_info;


    pluginlib::ClassLoader<seed_converter::StrokeConverter> converter_loader_;
    boost::shared_ptr<seed_converter::StrokeConverter> stroke_converter_ = nullptr;

    std::atomic<bool> shutdown = false;
    std::atomic<bool> cancel = false;
    std::thread send_thread;
    std::mutex mtx;
    std::condition_variable cond;

    MotionBase* cur_motion = nullptr;
    MotionIterator::Ptr cur_miter = nullptr;
    JointTarget jtarget;

    double dt_sec = 0.02;
    int start_step = 0;


    using EXT_PARAMS = std::vector<ExtensionParam>;
    using EXTENSION_STEP_AND_PARAMS = std::pair<int,EXT_PARAMS>;
    using EXTENSION_PARAM_LIST = std::queue<EXTENSION_STEP_AND_PARAMS>;
    int EXTENSION_STEP_AND_PARAMS::* EXTENSION_STEP = &EXTENSION_STEP_AND_PARAMS::first;
    EXT_PARAMS EXTENSION_STEP_AND_PARAMS::* EXTENSION_PARAMS = &EXTENSION_STEP_AND_PARAMS::second;

    std::mutex extension_mtx;
    EXTENSION_PARAM_LIST extension_params;

    std::atomic<bool>* stop_request = nullptr;

    ExtensionList* extension_list = nullptr;
};
}
