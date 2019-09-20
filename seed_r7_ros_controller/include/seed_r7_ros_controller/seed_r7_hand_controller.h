#ifndef _HAND_CONTROLLER_H_
#define _HAND_CONTROLLER_H_

#include "seed_r7_ros_controller/seed_r7_robot_hardware.h"
#include <seed_r7_ros_controller/HandControl.h>


namespace robot_hardware
{

class HandController
{
public: 
  HandController(const ros::NodeHandle& _nh, robot_hardware::RobotHW *_in_hw);
  ~HandController();

  bool HandControlCallback(seed_r7_ros_controller::HandControl::Request& _req, seed_r7_ros_controller::HandControl::Response& _res); 

private: 
  ros::ServiceServer grasp_control_server_;
  robot_hardware::RobotHW *hw_;
  ros::NodeHandle nh_;

  int right_number_;
  int left_number_;

  const uint16_t SCRIPT_GRASP = 2;
  const uint16_t SCRIPT_UNGRASP = 3;
  const uint16_t SCRIPT_CANCEL = 4;


};

}

#endif
