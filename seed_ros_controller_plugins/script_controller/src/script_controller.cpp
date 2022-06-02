#include <cmath>
#include <script_controller.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace script_controller {

ScriptController::ScriptController(){

}

bool ScriptController::init(hardware_interface::JointScriptInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    service_server = controller_nh.advertiseService("run_script", &ScriptController::RunScriptRequestCallback, this);

    joint_names = hw->getNames();
    int nj = joint_names.size();
    handles.resize(nj);
    for(int idx = 0;idx < nj;++idx){
        handles[idx] = hw->getHandle(joint_names[idx]);
    }

    script_info.script_no.resize(nj);

    return true;
}

void ScriptController::update(const ros::Time &time, const ros::Duration &period) {
    ScriptInfo *info = (script_buf_.readFromRT());
    if(info->executed || joint_names.size() != info->script_no.size()){
        return;
    }

    for (size_t idx = 0; idx < joint_names.size(); ++idx) {
        handles[idx].setCommand(info->script_no[idx]);
        info->executed = true;
    }
}

void ScriptController::starting(const ros::Time &time) {
}

void ScriptController::stopping(const ros::Time &time) {
}

bool ScriptController::RunScriptRequestCallback(script_controller::RunScript::Request &_req, script_controller::RunScript::Response &_res) {
    if (_req.joint_name.size() != _req.script_number.size()) {
        return false;
    }

    script_info.executed = false;
    for (size_t idx = 0;idx < joint_names.size();++idx) {
        script_info.script_no[idx] = 0x7F;
        for (size_t idx2 = 0; idx2 < _req.joint_name.size(); ++idx2) {
            if(_req.joint_name[idx2] == joint_names[idx]){
                script_info.script_no[idx] =  _req.script_number[idx2];
                break;
            }
        }
    }

    script_buf_.writeFromNonRT(script_info);

    return true;
}

}

PLUGINLIB_EXPORT_CLASS(script_controller::ScriptController, controller_interface::ControllerBase)
