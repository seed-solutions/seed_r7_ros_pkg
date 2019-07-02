#ifndef NOID_GRASP_H_
#define NOID_GRASP_H_

#include <vector>
#include <string>
#include <fstream>

#include <ros/ros.h>
//include seed_solutions_sdk
//#include "noid_robot_hardware.hh"

namespace noid
{
namespace grasp
{

class NoidGrasp
{
 public: typedef boost::shared_ptr< NoidGrasp> Ptr;

 public: NoidGrasp(const ros::NodeHandle& _nh,
                   noid_robot_hardware::NoidRobotHW *_in_hw) : nh_(_nh), hw_(_in_hw)
  {
    grasp_control_server_ =
      nh_.advertiseService(
        "grasp_control",
        &NoidGrasp::GraspControlCallback,
        this);
  }

 public: ~NoidGrasp() {} ;

 public: bool GraspControlCallback(noid_ros_controller::GraspControl::Request&  _req,
                                   noid_ros_controller::GraspControl::Response& _res) {
   // hw_->stopUpper(); // not needed

   ROS_WARN("NoidGrasp: Grasp pos: %d, script %d, power: %d",
            _req.position, _req.script, _req.power);

   // return if cancel script
   if (_req.script == noid_ros_controller::GraspControlRequest::SCRIPT_CANCEL) {
     ROS_WARN("AeroGrasp: setMaxSingleCurrent");
     hw_->setMaxSingleCurrent(_req.position, _req.power);
     ROS_WARN("AeroGrasp: handscript cancel");
     hw_->handScript(_req.position, _req.script);
     // hw_->startUpper(); // not needed
     return true;
   } else if (_req.script == noid_ros_controller::GraspControlRequest::SCRIPT_GRASP) {
     ROS_WARN("NoidGrasp: setMaxSingleCurrent");
     hw_->setMaxSingleCurrent(_req.position, _req.power);
     ROS_WARN("NoidGrasp: handScript grasp");
     hw_->handScript(_req.position, _req.script);
     ros::Duration(2.8).sleep(); // wait 2.8 seconds, as script takes max 2.8 seconds!
   } else if (_req.script == noid_ros_controller::GraspControlRequest::SCRIPT_UNGRASP) {
     ROS_WARN("NoidGrasp: handScript ungrasp");
     hw_->handScript(_req.position, _req.script);
     ros::Duration(2.7).sleep(); // wait 2.7 seconds, as script takes max 2.7 seconds!
   } else if (_req.script == noid_ros_controller::GraspControlRequest::COMMAND_SERVO) {
     ROS_WARN("NoidGrasp: cancel step-out");
     hw_->servo(_req.position);
     ROS_WARN("NoidGrasp: setMaxSingleCurrent"); // just in case
     hw_->setMaxSingleCurrent(_req.position, (100 << 8) + 30);
   }

   ROS_WARN("AeroGrasp: End Grasp");
   // !!!!!!!!!!!!!!! not supported
   // _res.angles.resize(2);
   // _res.angles[0] = upper_angles[13];
   // _res.angles[1] = upper_angles[27];
   // hw_->startUpper(); // not needed
   return true;
 }

  /// @param node handle
 private: ros::NodeHandle nh_;

 private: ros::ServiceServer grasp_control_server_;
  ///
 private: noid_robot_hardware::NoidRobotHW *hw_;

};

}  // grasp
}  // aero

#endif
