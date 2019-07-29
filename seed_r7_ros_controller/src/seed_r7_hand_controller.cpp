#include <seed_r7_hand_controller.h>


HandController::HandController(const ros::NodeHandle& _nh, robot_hardware::RobotHW *_in_hw)
:right_number_(0),left_number_(0)
{
  ROS_INFO("hand_control_server start");
 
  hw_ = _in_hw;
  nh_ = _nh;
  grasp_control_server_ = nh_.advertiseService("hand_control", &HandController::HandControlCallback,this);

  if(nh_.hasParam("/joint_settings/hand/right_number")) nh_.getParam("/joint_settings/hand/right_number", right_number_);
  if(nh_.hasParam("/joint_settings/hand/left_number")) nh_.getParam("/joint_settings/hand/left_number", left_number_);

  //initialize script cancel on right_hand
  hw_->runHandScript(right_number_, SCRIPT_CANCEL,0);
  //initialize script cancel on left_hand
  hw_->runHandScript(left_number_, SCRIPT_CANCEL,0);
    
  ROS_INFO("Initialized Handcontroller");
}   

HandController::~HandController()
{

}

bool HandController::HandControlCallback(seed_r7_ros_controller::HandControl::Request&  _req,
                                   seed_r7_ros_controller::HandControl::Response& _res) 
{
  uint8_t send_number;
  uint16_t script_number;

  ROS_INFO("Grasp callback start");

  //position
  if(_req.position == "right") send_number = right_number_;
  else if(_req.position == "left")  send_number = left_number_;
  else {
    ROS_ERROR("please input \"right\" or \"left\". ");
    _res.result = "service call failed";
    return false;
  }

  //script
  if(_req.script == "grasp") script_number = SCRIPT_GRASP;
  else if(_req.script == "ungrasp") script_number = SCRIPT_UNGRASP;
  else if(_req.script == "cancel") script_number = SCRIPT_CANCEL;
  else {
    ROS_ERROR("please input \"grasp\", \"ungrasp\" or \"cancel\".");
    _res.result = "service call failed";
    return false;
  }

  ROS_INFO("motion: %s", _req.script.c_str());
  hw_->runHandScript(send_number, script_number, _req.current);

  ROS_INFO("End Grasp");
  return true;
}
