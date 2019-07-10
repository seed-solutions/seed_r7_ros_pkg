#ifndef NOID_GRASP_H_
#define NOID_GRASP_H_

#include <vector>
#include <string>
#include <fstream>
#include <mutex>

#include <ros/ros.h>

#include "noid_robot_hardware.h"

#include <noid_ros_controller/GraspControl.h>

namespace noid
{
namespace grasp
{

class NoidHandControl
{
public: 
    NoidHandControl(const ros::NodeHandle& _nh, noid_robot_hardware::NoidRobotHW *_in_hw);
  
    ~NoidHandControl();
 
    bool GraspControlCallback(noid_ros_controller::GraspControl::Request&  _req,
                              noid_ros_controller::GraspControl::Response& _res); 
    void SelectHandScript(std::string _motion, int16_t _hand_type, int16_t _hand_scriptnum, int16_t _power);

  /// @param node handle
private: 
 ros::ServiceServer grasp_control_server_;
 //@param
 int hand_right_num_;
 int hand_left_num_;

 noid_robot_hardware::NoidRobotHW *hw_;

 ros::NodeHandle nh_;

 const std::vector<std::string > rhand_joints = { "r_thumb_joint" };
 const std::vector<std::string > lhand_joints = { "l_thumb_joint" };

 const int16_t script_grasp = 2;
 const int16_t script_ungrasp = 3;
 const int16_t script_cancel = 4;


};

}  // grasp
}  // noid

#endif
