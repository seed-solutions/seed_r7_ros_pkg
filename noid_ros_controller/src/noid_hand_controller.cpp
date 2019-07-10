#include <ros/ros.h>
#include <noid_ros_controller/GraspControl.h>

#include <noid_hand_controller.h>

using namespace noid_ros_controller;
using namespace noid;
using namespace grasp;


NoidHandControl::NoidHandControl(const ros::NodeHandle& _nh, 
                                noid_robot_hardware::NoidRobotHW *_in_hw):hand_right_num_(0),hand_left_num_(0)
{
  ROS_INFO("grasp_server_start");

  hw_ = _in_hw;
  nh_ = _nh;
  grasp_control_server_ = nh_.advertiseService("hand_control",
                                                &NoidHandControl::GraspControlCallback,this);

  if(nh_.hasParam("/noid_hand_controller/right_hand")) 
  {
    nh_.getParam("/noid_hand_controller/right_hand", hand_right_num_);
  }
  if(nh_.hasParam("/noid_hand_controller/left_hand")) 
  {
    nh_.getParam("/noid_hand_controller/left_hand", hand_left_num_);
  }

  //initialize script cancel on right_hand
  hw_->handScript(hand_right_num_, script_cancel);
  //initialize script cancel on left_hand
  hw_->handScript(hand_left_num_, script_cancel);
    
  ROS_INFO("Initialized Handcontroller");
}   

NoidHandControl::~NoidHandControl()
{

}

bool NoidHandControl::GraspControlCallback(noid_ros_controller::GraspControl::Request&  _req,
                                   noid_ros_controller::GraspControl::Response& _res) 
{
   ROS_INFO("Grasp callback start");
   
   if(_req.position == "right")
   {
           // return if cancel script
      if (_req.script == "cancel") {
        SelectHandScript(_req.script, hand_right_num_ ,script_cancel, _req.power);
      } 
      else if (_req.script == "grasp") {
        SelectHandScript(_req.script, hand_right_num_ ,script_grasp, _req.power);
        ros::Duration(2.8).sleep(); // wait 2.8 seconds, as script takes max 2.8 seconds!
      } 
      else if (_req.script == "ungrasp") {
        SelectHandScript(_req.script, hand_right_num_ ,script_ungrasp, _req.power);
        ros::Duration(2.7).sleep(); // wait 2.7 seconds, as script takes max 2.7 seconds!
      }
      else
      {
        ROS_ERROR("please input string_type or valid name");
        _res.result = "service call failed";
      }
   }
   else if(_req.position == "left")
   {
      // return if cancel script
      if (_req.script == "cancel") {
        SelectHandScript(_req.script, hand_left_num_ ,script_cancel, _req.power);
      } else if (_req.script == "grasp") {
        SelectHandScript(_req.script, hand_left_num_ ,script_grasp, _req.power);
        ros::Duration(2.8).sleep(); // wait 2.8 seconds, as script takes max 2.8 seconds!
      } else if (_req.script == "ungrasp") {
        SelectHandScript(_req.script, hand_left_num_ ,script_ungrasp, _req.power);
        ros::Duration(2.7).sleep(); // wait 2.7 seconds, as script takes max 2.7 seconds!
      }
      else
      {
        ROS_ERROR("please input string_type or valid name");
        _res.result = "service call failed";
      }
   }
   ROS_INFO("End Grasp");
   return true;
   
 
}

void NoidHandControl::SelectHandScript(std::string _motion, int16_t _hand_type, int16_t _hand_scriptnum, int16_t _power)
{
      ROS_INFO("handscript: setMaxSingleCurrent");
      //hw_->setMaxSingleCurrent(_hand_type, _power);
      ROS_INFO("motion: %s", _motion.c_str());
      hw_->handScript(_hand_type, _hand_scriptnum);
  
}



