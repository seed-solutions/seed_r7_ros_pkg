#include <pluginlib/class_list_macros.hpp>
#include "diagnostic_controller.hpp"

namespace diagnostic_controller {

bool DiagnosticController::init(hardware_interface::StatusInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    auto names = hw->getNames();
    if(names.size() != 1){
        ROS_ERROR_STREAM("robot status handle was not specified.");
        return false;
    }
    handle = hw->getHandle(names[0]);


    diagnostic_updater_.setHardwareID("SEED-Noid-Mover");
    diagnostic_updater_.add("RobotStatus",this,&DiagnosticController::setDiagnostics);

    timer = controller_nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent& e){diagnostic_updater_.update();});

    client = root_nh.serviceClient<aero_controller::ServoOn>("aero_controller/servo_on");

    return true;
}

DiagnosticController::~DiagnosticController(){
}

void DiagnosticController::starting(const ros::Time &time) {
}

void DiagnosticController::update(const ros::Time &time, const ros::Duration &period) {
    auto status = handle.getCommandPtr();
    swp_buff_recv.writeFromA(*status);
}

void DiagnosticController::stopping(const ros::Time &time) {
}

void DiagnosticController::setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
    struct MSSummary {
        union {
            SingleCANErrorInfo info;
            uint8_t info_u8;
        };
    };

    Status *cur_status = swp_buff_recv.readFromB();
    if (!cur_status) {
        return;
    }

    bool usb_err = 0;
    uint16_t ms_err = 0;

    for (int idx = 0; idx < cur_status->usb_num; ++idx) {
        usb_err |= cur_status->usb_status[idx].disconnect;
    }

    for (int idx = 0; idx < cur_status->ms_num; ++idx) {
        ms_err |= cur_status->ms_status[idx].err_u16;
    }

    MSSummary ms_sum;
    ms_sum.info_u8 = reinterpret_cast<uint8_t*>(&ms_err)[0] | reinterpret_cast<uint8_t*>(&ms_err)[1];
    bool estop = ms_sum.info.connection && ms_sum.info.calibration;

    if (estop) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "E-Stop switch is pushed, please release it");
    } else if (usb_err) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Now calibrating, or the USB cable is plugged out");
    } else if (ms_sum.info.power) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Power failed, please check the battery");
    } else if (ms_sum.info.connection) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Connection error occurred, please check the cable");
    } else if (ms_sum.info.temperature) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor driver is high temperature, please reboot the robot");
    } else if (ms_sum.info.protective_stopped) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Protective stopped, please release it");
    } else if (ms_sum.info.calibration) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Calibration error occurred, please re-calibration");
    } else if (ms_sum.info.step_out) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Step-out has occurred");
    } else if (ms_sum.info.response) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Response error has occurred");
    } else if (ms_sum.info.motor_stat) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Motor servo is off");
    } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "System all green");
    }

    stat.add("Emergency Stopped", estop);
    stat.add("Communication Error", usb_err);
    stat.add("Power Failed", ms_sum.info.power);
    stat.add("Connection Error", ms_sum.info.connection);
    stat.add("Temperature Error", ms_sum.info.temperature);
    stat.add("Protective Stopped", ms_sum.info.protective_stopped);
    stat.add("Calibration Error", ms_sum.info.calibration);
    stat.add("Step Out Occurred", ms_sum.info.step_out);
    stat.add("Response Error", ms_sum.info.response);
    stat.add("Motor Servo OFF", ms_sum.info.motor_stat);

    if(ms_sum.info.motor_stat){
        aero_controller::ServoOn srv;
        client.call(srv);
    }


}

}

PLUGINLIB_EXPORT_CLASS(diagnostic_controller::DiagnosticController, controller_interface::ControllerBase)
