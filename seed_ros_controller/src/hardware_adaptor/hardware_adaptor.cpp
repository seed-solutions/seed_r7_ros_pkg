#include <thread>
#include "hardware_adaptor.hpp"


bool HardwareAdaptor::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)  // add joint list
        {

    driver = new RobotDriver(root_nh,robot_hw_nh);
    int numJoints = driver->getNumJoints();

    joint_position_.resize(numJoints,0.0);
    joint_velocity_.resize(numJoints,0.0);
    joint_effort_.resize(numJoints,0.0);
    joint_position_command_.resize(numJoints,std::numeric_limits<double>::quiet_NaN());
    joint_velocity_command_.resize(numJoints,std::numeric_limits<double>::quiet_NaN());
    script_command_.resize(numJoints,0x7FFF);

    for (int j = 0; j < numJoints; ++j) {
        std::string jointname = driver->getJointName(j);

        hardware_interface::JointStateHandle js_handle = hardware_interface::JointStateHandle(jointname, &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]);
        hardware_interface::JointHandle pos_control_handle = hardware_interface::JointHandle(js_handle, &joint_position_command_[j]);
        hardware_interface::JointHandle vel_control_handle = hardware_interface::JointHandle(js_handle, &joint_velocity_command_[j]);
        hardware_interface::ScriptHandle script_command_handle = hardware_interface::ScriptHandle(jointname, &script_command_[j]);

        js_interface_.registerHandle(js_handle);
        pj_interface_.registerHandle(pos_control_handle);
        vj_interface_.registerHandle(vel_control_handle);
        sc_interface_.registerHandle(script_command_handle);
    }

    int numPorts = driver->getNumPorts();
    int numMs = driver->getNumMs();
    other_cmd_recv.resize(numMs);
    other_cmd_send.resize(numMs);
    status.usb_num = numPorts;
    status.ms_num = numMs;

    if(numPorts > status.get_usb_capacity()){
        ROS_ERROR_STREAM("exceed usb capacity.");
    }

    if(numMs > status.get_ms_capacity()){
        ROS_ERROR_STREAM("exceed usb capacity.");
    }

    for (int idx = 0; idx < numMs; ++idx) {
        int msid = driver->getMsId(idx);
        status.ms_status[idx].msid = msid;
        hardware_interface::OtherCommandHandle oc_handle = hardware_interface::OtherCommandHandle(msid, &other_cmd_recv[idx], &other_cmd_send[idx]);
        oc_interface_.registerHandle(oc_handle);
    }
    hardware_interface::StatusHandle status_handle = hardware_interface::StatusHandle("status",&status);
    st_interface_.registerHandle(status_handle);

    hardware_interface::MovingTimeHandle mt_handle = hardware_interface::MovingTimeHandle("time",&tgt_time_sec);
    mt_interface_.registerHandle(mt_handle);

    registerInterface(&js_interface_);
    registerInterface(&pj_interface_);
    registerInterface(&vj_interface_);
    registerInterface(&sc_interface_);
    registerInterface(&oc_interface_);
    registerInterface(&st_interface_);
    registerInterface(&mt_interface_);

    return true;
}

HardwareAdaptor::~HardwareAdaptor() {
    if(driver){
        delete driver;
        driver = nullptr;
    }
}

void HardwareAdaptor::read(const ros::Time &time, const ros::Duration &period) {
    driver->read(time,period);
    driver->getVel(joint_velocity_);
    driver->getPos(joint_position_);
    driver->getStatus(status);
    driver->getOtherCommands(other_cmd_recv);
}

void HardwareAdaptor::write(const ros::Time &time, const ros::Duration &period) {
    driver->sendVelocity(joint_velocity_command_);
    driver->sendPosition(joint_position_command_,tgt_time_sec);
    driver->runScript(script_command_);
    driver->sendOtherCommands(other_cmd_send);
    driver->send();
    tgt_time_sec = 0;
}
