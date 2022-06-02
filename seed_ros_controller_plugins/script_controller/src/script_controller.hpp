#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "hardware_interface/script_command_interface.hpp"
#include <script_controller/RunScript.h>

namespace script_controller{

  class ScriptController : public controller_interface::Controller<hardware_interface::JointScriptInterface>
  {
  public:
    ScriptController();

    bool init(hardware_interface::JointScriptInterface*  hw,
               ros::NodeHandle& root_nh,
               ros::NodeHandle &controller_nh) override;

    void update(const ros::Time& time, const ros::Duration& period) override;

    void starting(const ros::Time& time) override;

    void stopping(const ros::Time& time) override;

  private:

    bool RunScriptRequestCallback(script_controller::RunScript::Request &_req, script_controller::RunScript::Response &_res);

  private:
    struct ScriptInfo {
        std::vector<uint8_t> script_no;
        bool executed;
    };

    realtime_tools::RealtimeBuffer<ScriptInfo> script_buf_;
    ScriptInfo script_info;

    ros::ServiceServer service_server;
    std::vector<hardware_interface::ScriptHandle>  handles;
    std::vector<std::string> joint_names;

  };

}
