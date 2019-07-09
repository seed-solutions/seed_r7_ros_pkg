#ifndef NOID_GRASP_H_
#define NOID_GRASP_H_

#include <vector>
#include <string>
#include <fstream>
#include <mutex>

#include <ros/ros.h>

#include "seed_solutions_sdk/aero3_command.h"
#include "noid_robot_hardware.h"

#include <noid_ros_controller/GraspControl.h>

namespace noid
{
namespace grasp
{

class NoidGrasp 
{
 public: typedef boost::shared_ptr< NoidGrasp> Ptr;

 public: NoidGrasp(const ros::NodeHandle& _nh,
                   noid_robot_hardware::NoidRobotHW *_in_hw) : nh_(_nh),  hw_(_in_hw)
  {
    ROS_INFO("grasp_server_start");
    grasp_control_server_ =
      nh_.advertiseService(
        "grasp_control",
        &NoidGrasp::GraspControlCallback,
        this);

  }

 public: ~NoidGrasp() {} ;


 public: bool GraspControlCallback(noid_ros_controller::GraspControl::Request&  _req,
                                   noid_ros_controller::GraspControl::Response& _res) 
{
 
   ROS_INFO("Grasp: Grasp pos: %d, script %d, power: %d",
            _req.position, _req.script, _req.power);

   // return if cancel script
   if (_req.script == noid_ros_controller::GraspControlRequest::SCRIPT_CANCEL) {
     ROS_INFO("handscript: setMaxSingleCurrent");
     //hw_->setMaxSingleCurrent(_req.position, _req.power);
     ROS_INFO("handscript: cancel");
     hw_->handScript(_req.position, _req.script);
     

     return true;
   } else if (_req.script == noid_ros_controller::GraspControlRequest::SCRIPT_GRASP) {
     ROS_INFO("handscript : setMaxSingleCurrent");
     //hand_->setMaxSingleCurrent(_req.position, _req.power);
     ROS_INFO("handscript : grasp");
     hw_->handScript(_req.position, _req.script);
     ros::Duration(2.8).sleep(); // wait 2.8 seconds, as script takes max 2.8 seconds!
   } else if (_req.script == noid_ros_controller::GraspControlRequest::SCRIPT_UNGRASP) {
     ROS_INFO("handscript: ungrasp");
     hw_->handScript(_req.script, _req.position);
     ros::Duration(2.7).sleep(); // wait 2.7 seconds, as script takes max 2.7 seconds!
   } else if (_req.script == noid_ros_controller::GraspControlRequest::COMMAND_SERVO) {
     ROS_INFO("cancel step-out");
     //hw_->servo(_req.position);
     ROS_INFO("handscript: setMaxSingleCurrent"); // just in case
     //hw_->setMaxSingleCurrent(_req.position, (100 << 8) + 30);
   }

   ROS_INFO("End Grasp");
   return true;
 }

  /// @param node handle
 private: ros::NodeHandle nh_;

 private: ros::ServiceServer grasp_control_server_;
  ///
 private: noid_robot_hardware::NoidRobotHW *hw_;
 

const std::vector<std::string > rhand_joints = { "r_thumb_joint" };
const std::vector<std::string > lhand_joints = { "l_thumb_joint" };





};

}  // grasp
}  // noid

#endif
