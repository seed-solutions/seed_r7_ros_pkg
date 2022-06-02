#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/status_interface.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <libs/swap_buffer.hpp>
#include <thread>

#include <aero_controller/ServoOn.h>

namespace diagnostic_controller {

class DiagnosticController: public controller_interface::Controller<hardware_interface::StatusInterface> {
public:
    DiagnosticController() {
    }

    ~DiagnosticController();

    bool init(hardware_interface::StatusInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
    void starting(const ros::Time &time) override;
    void update(const ros::Time &time, const ros::Duration &period) override;
    void stopping(const ros::Time &time) override;

private:
    void setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
private:
    hardware_interface::StatusHandle  handle;
    diagnostic_updater::Updater diagnostic_updater_;
    ros::Timer timer;

    SwapBuffer<Status, 1> swp_buff_recv;

    ros::ServiceClient client;
};
}
