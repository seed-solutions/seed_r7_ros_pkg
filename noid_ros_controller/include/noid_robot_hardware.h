/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Yohei Kakiuchi (JSK lab.)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 Author: Yohei Kakiuchi
*/

#ifndef _NOID_ROBOT_HW_H_
#define _NOID_ROBOT_HW_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>

// URDF
#include <urdf/model.h>

// AERO
#include "noid_upper_controller.h"
#include "noid_lower_controller.h"
#include "RobotStrokeConverter.h"
#include "UnusedAngle2Stroke.h"

#include <mutex>

using namespace noid;
using namespace controller;
using namespace common;

namespace noid_robot_hardware
{

class NoidRobotHW : public hardware_interface::RobotHW
{
public:
  NoidRobotHW() { }

  virtual ~NoidRobotHW() {}

  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  virtual void read(const ros::Time& time, const ros::Duration& period);
  virtual void write(const ros::Time& time, const ros::Duration& period);

  void readPos(const ros::Time& time, const ros::Duration& period, bool update);
  void writeWheel(const std::vector< std::string> &_names, const std::vector<int16_t> &_vel, double _tm_sec);
  double getPeriod() { return ((double)CONTROL_PERIOD_US_) / (1000 * 1000); }

protected:
  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};
  enum JointType {NONE, PRISMATIC, ROTATIONAL, CONTINUOUS, FIXED};

  unsigned int number_of_angles_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::PositionJointInterface pj_interface_;

  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;

  std::vector<std::string> joint_list_;
  std::vector<double> joint_effort_limits_;
  std::vector<JointType>     joint_types_;
  std::vector<ControlMethod> joint_control_methods_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;


  std::vector<double> prev_ref_positions_;
  std::vector<int16_t> upper_act_strokes_;
  std::vector<int16_t> lower_act_strokes_;

  boost::shared_ptr<NoidUpperController> controller_upper_;
  boost::shared_ptr<NoidLowerController> controller_lower_;

  bool initialized_flag_;
  bool upper_send_enable_;

  int   CONTROL_PERIOD_US_;
  float OVERLAP_SCALE_;
  int   BASE_COMMAND_PERIOD_MS_;

  std::mutex mutex_lower_;
  std::mutex mutex_upper_;

  std::vector<std::string> joint_names_upper_;
  std::vector<std::string> joint_names_lower_;
  std::string robot_model;

};

typedef boost::shared_ptr<NoidRobotHW> NoidRobotHWPtr;
}

#endif
