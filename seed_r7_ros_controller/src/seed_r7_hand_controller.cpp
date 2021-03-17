#include "seed_r7_ros_controller/seed_r7_hand_controller.h"


robot_hardware::HandController::HandController
(const ros::NodeHandle& _nh, robot_hardware::RobotHW *_in_hw)
  : right_number_(0),left_number_(0)
{
  ROS_INFO("hand_control_server start");
 
  hw_ = _in_hw;
  nh_ = _nh;
  grasp_control_server_
    = nh_.advertiseService("hand_control", &HandController::HandControlCallback,this);

  if (nh_.hasParam("/joint_settings/hand/right_number"))
    nh_.getParam("/joint_settings/hand/right_number", right_number_);
  if (nh_.hasParam("/joint_settings/hand/left_number"))
    nh_.getParam("/joint_settings/hand/left_number", left_number_);

  // initialize script cancel on right_hand
  if(right_number_ != 0) hw_->runHandScript(right_number_, SCRIPT_CANCEL,0);
  // initialize script cancel on left_hand
  if(left_number_ != 0) hw_->runHandScript(left_number_, SCRIPT_CANCEL,0);
    
  ROS_INFO("Initialized Handcontroller");
}   

robot_hardware::HandController::~HandController()
{
}

bool robot_hardware::HandController::HandControlCallback
(seed_r7_ros_controller::HandControl::Request&  _req,
 seed_r7_ros_controller::HandControl::Response& _res) 
{
  uint8_t send_number;
  uint16_t script_number;

  ROS_INFO("Grasp callback start");

  // position
  // number depends on configuration, therefore if/else check
  if (_req.position == seed_r7_ros_controller::HandControl::Request::POSITION_RIGHT)
    send_number = right_number_;
  else if (_req.position == seed_r7_ros_controller::HandControl::Request::POSITION_LEFT)
    send_number = left_number_;
  else {
    ROS_ERROR("please input POSITION_RIGHT(0) or POSITION_LEFT(1). ");
    _res.result = "service call failed";
    return false;
  }

  // script
  if (_req.script == seed_r7_ros_controller::HandControl::Request::SCRIPT_GRASP)
    script_number = SCRIPT_GRASP;
  else if (_req.script == seed_r7_ros_controller::HandControl::Request::SCRIPT_RELEASE)
    script_number = SCRIPT_UNGRASP;
  else if (_req.script == seed_r7_ros_controller::HandControl::Request::SCRIPT_CANCEL)
    script_number = SCRIPT_CANCEL;
  else {
    ROS_ERROR("please input \"grasp\", \"release\" or \"cancel\".");
    _res.result = "service call failed";
    return false;
  }

  ROS_INFO("motion: %s", _req.script.c_str());
  if(send_number != 0) hw_->runHandScript(send_number, script_number, _req.current);

  ROS_INFO("End Grasp");
  return true;
}
