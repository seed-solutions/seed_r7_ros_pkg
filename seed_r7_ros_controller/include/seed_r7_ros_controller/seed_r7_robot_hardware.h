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

#ifndef _ROBOT_HW_H_
#define _ROBOT_HW_H_

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
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <pluginlib/class_loader.h>

// URDF
#include <urdf/model.h>

// AERO
#include "seed_r7_ros_controller/seed_r7_upper_controller.h"
#include "seed_r7_ros_controller/seed_r7_lower_controller.h"
#include "seed_r7_ros_controller/stroke_converter_base.h"
#include "seed_r7_ros_controller/ResetRobotStatus.h"

#include <mutex>

//for robot status view
#include <std_msgs/String.h>
#include <diagnostic_updater/diagnostic_updater.h>

namespace robot_hardware
{

class RobotHW : public hardware_interface::RobotHW
{
public:
  RobotHW() : converter_loader_("seed_r7_ros_controller", "StrokeConverter"),is_stop(false),cyclic_stopped(false) { }

  virtual ~RobotHW() {}

  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  virtual void read(const ros::Time& time, const ros::Duration& period);
  virtual void write(const ros::Time& time, const ros::Duration& period);

  void readPos(const ros::Time& time, const ros::Duration& period, bool update);
  void writeWheel(const std::vector< std::string> &_names,
                  const std::vector<int16_t> &_vel, double _tm_sec);
  double getPeriod() { return ((double)CONTROL_PERIOD_US_) / (1000 * 1000); }


  void stopCyclic(bool is_stop){
        this->is_stop.store(is_stop);
        if (is_stop) {
            //停止するまで待つ
            while (!cyclic_stopped.load()) {
                usleep(100);
            }
        }else{
            //再開するまで待つ
            while (cyclic_stopped.load()) {
                usleep(100);
            }
        }
  }

  //--specific functions--
  void runHandScript(uint8_t _number, uint16_t _script, uint8_t _current);
  void turnWheel(std::vector<int16_t> &_vel);
  void onWheelServo(bool _value);
  void getBatteryVoltage(const ros::TimerEvent& _event);
  void runLedScript(uint8_t _number, uint16_t _script);
  void setRobotStatus();
  void setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  void write_1byte_lower(uint16_t _address, uint8_t *_write_data, int write_size);
  void write_1byte_upper(uint16_t _address, uint8_t *_write_data, int write_size);
  std::vector<uint8_t>  read_1byte_lower(uint16_t _address, int size);
  std::vector<uint8_t>  read_1byte_upper(uint16_t _address, int size);
  void resetting_lower();
  void resetting_upper();


  //TODO 以下、もしも上半身・下半身どちらも接続していてCosmoCMDを受け取る方を任意に切り替える必要があるなら実装変更の必要あり
  //現状は上半身がつながっていた時点で上半身側でCosmoCMDを処理する。
  CosmoCmdReqType getCosmoCmd(){
      if (controller_upper_->is_open_){
          return controller_upper_->getCosmoCmd();
      }
      else if(controller_lower_->is_open_){
          return controller_lower_->getCosmoCmd();
      }
  }

  void sendCosmoCmdResp(CosmoCmdRespType resp){
      if (controller_upper_->is_open_){
          controller_upper_->sendCosmoCmdResp(resp);
      }
      else if(controller_lower_->is_open_){
          controller_lower_->sendCosmoCmdResp(resp);
      }
  }

  RobotStatusCmdReqType getRobotStatusCmd(){
      if (controller_upper_->is_open_){
          return controller_upper_->getRobotStatusCmd();
      }
      else if(controller_lower_->is_open_){
          return controller_lower_->getRobotStatusCmd();
      }
  }

  void sendRobotStatusCmdResp(RobotStatusCmdRespType resp){
      if (controller_upper_->is_open_){
          controller_upper_->sendRobotStatusCmdResp(resp);
      }
      else if(controller_lower_->is_open_){
          controller_lower_->sendRobotStatusCmdResp(resp);
      }
  }


  VirtualControllerCmdReqType getVirtualControllerCmd(){
      //とりあえず今はlowerだけしか操作しないものとする
      if(controller_lower_->is_open_){
          return controller_lower_->getVirtualControllerCmd();
      }
  }

  //----------------------

  bool comm_err_;
  struct RobotStatus {
      bool connection_err_;
      bool calib_err_;
      bool motor_err_;
      bool temp_err_;
      bool res_err_;
      bool step_out_err_;
      bool p_stopped_err_;
      bool power_err_;
    } robot_status_;

private:
  ros::ServiceServer reset_robot_status_server_;
  bool resetRobotStatusCallback(seed_r7_ros_controller::ResetRobotStatus::Request& _req, seed_r7_ros_controller::ResetRobotStatus::Response& _res);

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


  std::vector<double> prev_ref_strokes_;
  std::vector<int16_t> upper_act_strokes_;
  std::vector<int16_t> lower_act_strokes_;

  boost::shared_ptr<robot_hardware::UpperController> controller_upper_;
  boost::shared_ptr<robot_hardware::LowerController> controller_lower_;

  bool initialized_flag_;
  bool upper_send_enable_;

  int   CONTROL_PERIOD_US_;
  float OVERLAP_SCALE_;
  int   BASE_COMMAND_PERIOD_MS_;

  std::mutex mutex_lower_;
  std::mutex mutex_upper_;

  std::vector<std::string> joint_names_upper_;
  std::vector<std::string> joint_names_lower_;
  std::string robot_model_plugin_;

  ros::Timer bat_vol_timer_;
  ros::Publisher bat_vol_pub_;

  ros::Publisher joy_pub_;
  sensor_msgs::Joy joy_;

  pluginlib::ClassLoader<seed_converter::StrokeConverter> converter_loader_;
  boost::shared_ptr<seed_converter::StrokeConverter> stroke_converter_;

  //for robot status view
  diagnostic_updater::Updater diagnostic_updater_;

  std::atomic<bool> cyclic_stopped;
  std::atomic<bool> is_stop;
};

}

#endif
