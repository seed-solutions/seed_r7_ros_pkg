#ifndef _NOID_HAND_CONTROLLER_H_
#define _NOID_HAND_CONTROLLER_H_

#include "noid_robot_hardware.h"
#include <noid_ros_controller/HandControl.h>

namespace noid
{
namespace grasp
{

class NoidHandController
{
public: 
  NoidHandController(const ros::NodeHandle& _nh, noid_robot_hardware::NoidRobotHW *_in_hw);
  ~NoidHandController();

  bool HandControlCallback(noid_ros_controller::HandControl::Request& _req, noid_ros_controller::HandControl::Response& _res); 

private: 
  ros::ServiceServer grasp_control_server_;
  noid_robot_hardware::NoidRobotHW *hw_;
  ros::NodeHandle nh_;

  int right_number_;
  int left_number_;

  const uint16_t SCRIPT_GRASP = 2;
  const uint16_t SCRIPT_UNGRASP = 3;
  const uint16_t SCRIPT_CANCEL = 4;


};

}  // grasp
}  // noid

#endif
