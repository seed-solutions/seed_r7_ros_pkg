#pragma once

#include <hardware_interface/other_command_interface_helper.hpp>
#include "aero_data.hpp"
#include "aero_controller/ServoOn.h"

namespace aero_controller{

  class AeroController
      : public controller_interface::OtherCommandInterfaceHelper
  {
  public:
    AeroController();

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  private:
    bool servoOn(aero_controller::ServoOn::Request &req, aero_controller::ServoOn::Response &res);


  private:
    ros::ServiceServer svon_srv;
  };

}
