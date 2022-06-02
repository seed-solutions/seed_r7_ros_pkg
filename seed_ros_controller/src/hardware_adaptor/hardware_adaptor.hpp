#pragma once

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>


#include "robot_driver.hpp"
#include "other_cmd_buff.hpp"

#include "script_command_interface.hpp"
#include "other_command_interface.hpp"
#include "status_interface.hpp"
#include "moving_time_interface.hpp"


#include "robot_status.hpp"

class HardwareAdaptor : public hardware_interface::RobotHW
{
public:
  HardwareAdaptor() { }

  ~HardwareAdaptor();

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

  void readPos(const ros::Time& time, const ros::Duration& period, bool update);

private:
  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;
  hardware_interface::JointScriptInterface   sc_interface_;
  hardware_interface::OtherCommandInterface  oc_interface_;
  hardware_interface::StatusInterface        st_interface_;
  hardware_interface::MovingTimeInterface    mt_interface_;


  double tgt_time_sec = 0; //<! 位置司令時の到達目標時間 通常はゼロ指定
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<uint16_t> script_command_;
  std::vector<BuffList> other_cmd_recv;
  std::vector<BuffList> other_cmd_send;
  Status status;

  RobotDriver* driver = nullptr;
};
