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

#include <thread>
#include "seed_r7_ros_controller/seed_r7_robot_hardware.h"


namespace robot_hardware
{

  bool RobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)  // add joint list
  {
    std::string port_upper;
    std::string port_lower;

    // reading paramerters
    robot_hw_nh.param<std::string>("port_upper", port_upper, "/dev/aero_upper");
    robot_hw_nh.param<std::string>("port_lower", port_lower, "/dev/aero_lower");
    robot_hw_nh.param<std::string>("robot_model_plugin", robot_model_plugin_,
                                   "seed_r7_robot_interface/TypeF");
    //
    if (robot_hw_nh.hasParam("/joint_settings/upper"))
      robot_hw_nh.getParam("/joint_settings/upper/joints", joint_names_upper_);
    else
      ROS_WARN("/joint_settings/upper read error");
    //
    if (robot_hw_nh.hasParam("/joint_settings/lower"))
      robot_hw_nh.getParam("/joint_settings/lower/joints", joint_names_lower_);
    else
      ROS_WARN("/joint_settings/lower read error");
    //
    if (robot_hw_nh.hasParam("controller_rate")) {
      double rate;
      robot_hw_nh.getParam("controller_rate", rate);
      CONTROL_PERIOD_US_ = (1000*1000)/rate;
    } else {
      CONTROL_PERIOD_US_ = 50*1000;  // 50ms
    }
    if (robot_hw_nh.hasParam("overlap_scale")) {
      double scl;
      robot_hw_nh.getParam("overlap_scale", scl);
      OVERLAP_SCALE_    = scl;
    } else {
      OVERLAP_SCALE_    = 2.8;
    }

    if (root_nh.hasParam("/seed_r7_mover_controller/wheel_vel_limit")) {
      root_nh.getParam("/seed_r7_mover_controller/wheel_vel_limit", wheel_vel_limit_);
    } else {
      wheel_vel_limit_ = 9.0;
    }
    if (root_nh.hasParam("/seed_r7_mover_controller/pub_robot_info")) {
      root_nh.getParam("/seed_r7_mover_controller/pub_robot_info", pub_robot_info_);
    } else {
      pub_robot_info_ = false;
    }

    
    ROS_INFO("upper_port: %s", port_upper.c_str());
    ROS_INFO("lower_port: %s", port_lower.c_str());
    ROS_INFO("cycle: %f [ms], overlap_scale %f", CONTROL_PERIOD_US_*0.001, OVERLAP_SCALE_);

    // create controllers
    controller_upper_.reset(new robot_hardware::UpperController(port_upper));
    controller_lower_.reset(new robot_hardware::LowerController(port_lower));

    upper_connected_ = controller_upper_->is_open_;
    lower_connected_ = controller_lower_->is_open_;

    // load stroke converter
    // converter is dependent per robot strucutre, therefore uses plugin
    stroke_converter_ = converter_loader_.createInstance(robot_model_plugin_);
    if (!stroke_converter_->initialize(robot_hw_nh)) {
      ROS_ERROR("Failed to initiate stroke converter");
      return false;
    }

    number_of_angles_ = joint_names_upper_.size() + joint_names_lower_.size();

    joint_list_.resize(number_of_angles_);
    for (int i = 0; i < number_of_angles_; ++i) {
      if (i < joint_names_upper_.size()) joint_list_[i] = joint_names_upper_[i];
      else joint_list_[i] = joint_names_lower_[i - joint_names_upper_.size()];
    }

    prev_ref_strokes_.resize(number_of_angles_);
    initialized_flag_ = false;

    std::string model_str;
    if (!root_nh.getParam("robot_description", model_str)) {
      ROS_ERROR("Failed to get model from robot_description");
      return false;
    }
    urdf::Model model;
    if (!model.initString(model_str)) {
      ROS_ERROR("Failed to parse robot_description");
      return false;
    }

    ROS_DEBUG("read %d joints", number_of_angles_);
    for (int i = 0; i < number_of_angles_; ++i) {
      ROS_DEBUG("  %d: %s", i, joint_list_[i].c_str());
      if (!model.getJoint(joint_list_[i])) {
        ROS_ERROR("Joint %s does not exist in urdf model", joint_list_[i].c_str());
        return false;
      }
    }

    joint_types_.resize(number_of_angles_);
    joint_control_methods_.resize(number_of_angles_);
    joint_position_.resize(number_of_angles_);
    joint_velocity_.resize(number_of_angles_);
    joint_effort_.resize(number_of_angles_);
    joint_position_command_.resize(number_of_angles_);
    joint_velocity_command_.resize(number_of_angles_);
    joint_effort_command_.resize(number_of_angles_);

    wheel_angles_.resize(4);
    fill(wheel_angles_.begin(),wheel_angles_.end(),0);
    wheel_velocities_.resize(4);
    fill(wheel_velocities_.begin(),wheel_velocities_.end(),0);

    // Get Robot Firmware Version
    std::string upper_firmware = controller_upper_->getFirmwareVersion();
    std::string lower_firmware = controller_lower_->getFirmwareVersion();
    ROS_INFO("Upper Firmware Ver. is [ %s ]", upper_firmware.c_str());
    ROS_INFO("Lower Firmware Ver. is [ %s ]", lower_firmware.c_str());

    //----- robot info
    if(pub_robot_info_)
    {
      robot_info_timer_ = robot_hw_nh.createTimer(ros::Duration(0.1), &RobotHW::pubRobotInfo, this);
      robot_info_pub_ = robot_hw_nh.advertise<seed_r7_ros_controller::RobotInfo>("robot_info", 100);
/*
      mutex_lower_.lock();
      controller_lower_->stopPolling();
      robot_info_.driver[0].firmware = controller_lower_->getFirmwareVersion(3);
      robot_info_.driver[1].firmware = controller_lower_->getFirmwareVersion(4);
      robot_info_.driver[2].firmware = controller_lower_->getFirmwareVersion(5);
      robot_info_.driver[3].firmware = controller_lower_->getFirmwareVersion(6);
      mutex_lower_.unlock();
*/
      robot_info_.robot.firmware = lower_firmware;
    }

    cmd_vel_sub_ = root_nh.subscribe("cmd_vel", 1, &RobotHW::cmdVelCallback, this);
    odom_sub_ = root_nh.subscribe("odom", 1, &RobotHW::odomCallback, this);
    //--------

    readPos(ros::Time::now(), ros::Duration(0.0), true);  // initial

    // Initialize values
    for (unsigned int j = 0; j < number_of_angles_; ++j) {
      joint_velocity_[j] = 0.0;
      joint_velocity_command_[j] = 0.0;
      joint_effort_[j]   = 0.0;  // N/m for continuous joints
      joint_effort_command_[j]   = 0.0;

      std::string jointname = joint_list_[j];
      // Create joint state interface for all joints
      js_interface_.registerHandle
        (hardware_interface::JointStateHandle
         (jointname, &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

      joint_control_methods_[j] = ControlMethod::POSITION;
      hardware_interface::JointHandle joint_handle
        = hardware_interface::JointHandle(js_interface_.getHandle(jointname),
                                          &joint_position_command_[j]);
      pj_interface_.registerHandle(joint_handle);

      joint_limits_interface::JointLimits limits;
      const bool urdf_limits_ok
        = joint_limits_interface::getJointLimits(model.getJoint(jointname), limits);
      if (!urdf_limits_ok) {
        ROS_WARN("urdf limits of joint %s is not defined", jointname.c_str());
      }
      // Register handle in joint limits interface (Limits spec)
      joint_limits_interface::PositionJointSaturationHandle limits_handle(joint_handle,limits);
      pj_sat_interface_.registerHandle(limits_handle);
    }
    // Register interfaces
    registerInterface(&js_interface_);
    registerInterface(&pj_interface_);

    // Battery Voltage Publisher
    bat_vol_pub_ = robot_hw_nh.advertise<std_msgs::Float32>("voltage", 1);
    bat_vol_timer_ = robot_hw_nh.createTimer(ros::Duration(1), &RobotHW::getBatteryVoltage,this);

    //Robot Status
    reset_robot_status_server_
      = root_nh.advertiseService("reset_robot_status", &RobotHW::resetRobotStatusCallback,this);
    robot_status_.p_stopped_err_ = false;

    //robot status view
    pre_diag_level_ = 0;
    pre_diag_msg_ = "";
    diagnostic_updater_.setHardwareID("SEED-Noid-Mover");
    diagnostic_updater_.add("RobotStatus",this,&RobotHW::setDiagnostics);

    return true;
  }

  void RobotHW::readPos(const ros::Time& time, const ros::Duration& period, bool update)
  {
    mutex_lower_.lock();
    mutex_upper_.lock();
    if (update) {
      std::thread t1([&](){
          controller_upper_->getPosition();
        });
      std::thread t2([&](){
          controller_lower_->getPosition();
        });
      t1.join();
      t2.join();
    }
    mutex_upper_.unlock();
    mutex_lower_.unlock();

    // whole body strokes
    std::vector<int16_t> act_strokes(0);
    std::vector<int16_t> act_upper_strokes;
    std::vector<int16_t> act_lower_strokes;

    //remap
    controller_upper_->remapAeroToRos(act_upper_strokes, controller_upper_->raw_data_);
    controller_lower_->remapAeroToRos(act_lower_strokes, controller_lower_->raw_data_);

    act_strokes.insert(act_strokes.end(),act_upper_strokes.begin(),act_upper_strokes.end());
    act_strokes.insert(act_strokes.end(),act_lower_strokes.begin(),act_lower_strokes.end());

    // whole body positions from strokes
    std::vector<double> act_positions;
    act_positions.resize(number_of_angles_);

    // convert from stroke to angle
    stroke_converter_->Stroke2Angle(act_positions, act_strokes);

    double tm = period.toSec();
    for(unsigned int j=0; j < number_of_angles_; j++) {
      float position = act_positions[j];
      float velocity = 0.0;

      joint_position_[j] = position;
      joint_velocity_[j] = velocity; // read velocity from HW
      joint_effort_[j]   = 0;        // read effort   from HW
    }

    if (!initialized_flag_) {
      for(unsigned int j = 0; j < number_of_angles_; j++) {
        joint_position_command_[j] = joint_position_[j];
        ROS_DEBUG("%d: %s - %f", j, joint_list_[j].c_str(), joint_position_command_[j]);
      }
      initialized_flag_ = true;
    }

    //check robot status flag
    setRobotStatus();

    //in case of error flag is high
    if(robot_status_.p_stopped_err_){
      ROS_WARN("The robot is protective stopped, please release it.");
    }
    if(robot_status_.connection_err_ && robot_status_.calib_err_){
      ROS_WARN("The robot is Emergency stopped, please release it.");
    }

    for (size_t i=0; i<wheel_angles_.size() ; i++)
    {
      if((abs(controller_lower_->wheel_angles_.at(i) - wheel_angles_.at(i)) / period.toSec()) < wheel_vel_limit_)
        wheel_velocities_.at(i) = (controller_lower_->wheel_angles_.at(i) - wheel_angles_.at(i)) / period.toSec();
    }

    // noise filter
    int count_zero_vel = std::count(wheel_velocities_.begin(), wheel_velocities_.end(), 0);
    if(count_zero_vel == 3)
      fill(wheel_velocities_.begin(),wheel_velocities_.end(),0);

    wheel_angles_ = controller_lower_->wheel_angles_;

    return;
  }

  void RobotHW::read(const ros::Time& time, const ros::Duration& period)
  {
    return;
  }

  void RobotHW::write(const ros::Time& time, const ros::Duration& period)
  {
    pj_sat_interface_.enforceLimits(period);

    ////// convert positions to strokes and write strokes
    std::vector<double > ref_positions(number_of_angles_);
    for (unsigned int j = 0; j < number_of_angles_; ++j) {
      switch (joint_control_methods_[j]) {
      case ControlMethod::POSITION:
        {
          ref_positions[j] = joint_position_command_[j];
        }
        break;
      case ControlMethod::VELOCITY:
        {
        }
        break;
      case ControlMethod::EFFORT:
        {
        }
        break;
      case ControlMethod::POSITION_PID:
        {
        }
        break;
      case ControlMethod::VELOCITY_PID:
        {
        }
        break;
      }  // switch
    }  // for

    std::vector<bool > mask_positions(number_of_angles_);
    std::fill(mask_positions.begin(), mask_positions.end(), true); // send if true

    // convert from angle to stroke
    std::vector<int16_t> ref_strokes(ref_positions.size());
    stroke_converter_->Angle2Stroke(ref_strokes, ref_positions);

    for (int i = 0; i < number_of_angles_; ++i) {
      double tmp = ref_strokes[i];
      if (tmp == prev_ref_strokes_[i]) {
        mask_positions[i] = false;
      }
      prev_ref_strokes_[i] = tmp;
    }
    
    // masking
    std::vector<int16_t> snt_strokes(ref_strokes);
    for (size_t i = 0; i < ref_strokes.size() ; ++i) {
      if (!mask_positions[i]) snt_strokes[i] = 0x7FFF;
    }

    // split strokes into upper and lower
    std::vector<int16_t> upper_strokes;
    std::vector<int16_t> lower_strokes;

    // remap
    if (controller_upper_->is_open_) controller_upper_->remapRosToAero(upper_strokes,snt_strokes);
    else controller_upper_->remapRosToAero(upper_strokes,ref_strokes);
    if (controller_lower_->is_open_) controller_lower_->remapRosToAero(lower_strokes,snt_strokes);
    else controller_lower_->remapRosToAero(lower_strokes,ref_strokes);

    uint16_t time_csec = static_cast<uint16_t>((OVERLAP_SCALE_ * CONTROL_PERIOD_US_)/(1000*10));

    mutex_lower_.lock();
    mutex_upper_.lock();
    {
      std::thread t1([&](){
          controller_upper_->sendPosition(time_csec, upper_strokes);
        });
      std::thread t2([&](){
          controller_lower_->sendPosition(time_csec, lower_strokes);
        });
      t1.join();
      t2.join();
    }
    mutex_upper_.unlock();
    mutex_lower_.unlock();

    // read
    readPos(time, period, false);
    return;
  }

/////////////////////////////////////
// specific functions are below:  ///
/////////////////////////////////////
  void RobotHW::runHandScript(uint8_t _number, uint16_t _script, uint8_t _current)
  {
    mutex_upper_.lock();
    if(_script == 2){
      controller_upper_->runScript(_number, 4);
      usleep(20 * 1000);    //magic number
      controller_upper_->setCurrent(_number, _current, _current);
    }
    controller_upper_->runScript(_number, _script);
    ROS_INFO("sendnum : %d, script : %d", _number, _script);
    mutex_upper_.unlock();
  }

  void RobotHW::turnWheel(std::vector<int16_t> &_vel)
  {
    mutex_lower_.lock();
    controller_lower_->sendVelocity(_vel);
    mutex_lower_.unlock();
  }

   void RobotHW::onWheelServo(bool _value)
  {
    mutex_lower_.lock();
    controller_lower_->onServo(_value);
    mutex_lower_.unlock();
  }

  void RobotHW::getBatteryVoltage(const ros::TimerEvent& _event)
  {
    // max voltage is 26[V]
    // min voltage is 22.2[V]
    std_msgs::Float32 voltage;
    mutex_lower_.lock();
    std::vector<uint16_t> data = controller_lower_->getBatteryVoltage();
    mutex_lower_.unlock();
    voltage.data = data.at(30) / 10.0;
    bat_vol_pub_.publish(voltage);

    robot_info_.robot.voltage = data.at(30) / 10.0;

    robot_info_.driver[0].temp = static_cast<uint8_t>(data.at(2) >> 8);
    robot_info_.driver[1].temp = static_cast<uint8_t>(data.at(3) >> 8);
    robot_info_.driver[2].temp = static_cast<uint8_t>(data.at(4) >> 8);
    robot_info_.driver[3].temp = static_cast<uint8_t>(data.at(5) >> 8);
  }

  void RobotHW::pubRobotInfo(const ros::TimerEvent &_event)
  {
    // motor current
    std::vector<uint16_t> current;
    current.resize(31);

    mutex_lower_.lock();
    if (!robot_status_.connection_err_ && !robot_status_.calib_err_)
      current = controller_lower_->getMotorCurrent(0);
    else
      fill(current.begin(), current.end(), 0);
    mutex_lower_.unlock();
    robot_info_.driver[0].current = current.at(2);
    robot_info_.driver[1].current = current.at(3);
    robot_info_.driver[2].current = current.at(4);
    robot_info_.driver[3].current = current.at(5);

    // motor position
    robot_info_.driver[0].position = controller_lower_->wheel_angles_.at(0) * (180/M_PI);
    robot_info_.driver[1].position = controller_lower_->wheel_angles_.at(1) * (180/M_PI);
    robot_info_.driver[2].position = controller_lower_->wheel_angles_.at(2) * (180/M_PI);
    robot_info_.driver[3].position = controller_lower_->wheel_angles_.at(3) * (180/M_PI);

    // publish robot status
    robot_info_.header.stamp = ros::Time::now();
    robot_info_pub_.publish(robot_info_);
  }

  void RobotHW::runLedScript(uint8_t _number, uint16_t _script)
  {
    mutex_lower_.lock();
    controller_lower_->runScript(_number, _script);
    mutex_lower_.unlock();
  }

  void RobotHW::setRobotStatus()
  {
    comm_err_ = controller_lower_->comm_err_ || controller_upper_->comm_err_;

    robot_status_.connection_err_ = controller_lower_->robot_status_.connection_err_ ||
      controller_upper_->robot_status_.connection_err_;

    robot_status_.calib_err_ = controller_lower_->robot_status_.calib_err_ ||
      controller_upper_->robot_status_.calib_err_;

    robot_status_.motor_err_ = controller_lower_->robot_status_.motor_err_ ||
      controller_upper_->robot_status_.motor_err_;

    robot_status_.temp_err_ = controller_lower_->robot_status_.temp_err_ ||
      controller_upper_->robot_status_.temp_err_;

    robot_status_.res_err_ = controller_lower_->robot_status_.res_err_ ||
      controller_upper_->robot_status_.res_err_;

    robot_status_.step_out_err_ = controller_lower_->robot_status_.step_out_err_ ||
      controller_upper_->robot_status_.step_out_err_;

    robot_status_.p_stopped_err_ = controller_lower_->robot_status_.p_stopped_err_ ||
      controller_upper_->robot_status_.p_stopped_err_;

    robot_status_.power_err_ = controller_lower_->robot_status_.power_err_ ||
      controller_upper_->robot_status_.power_err_;

    diagnostic_updater_.update();

  }

  void RobotHW::setDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if(comm_err_ ){
      stat.summary(2,"Now calibrating, or the USB cable is plugged out");
    }
    else if(robot_status_.power_err_ ){
      stat.summary(2,"Power failed, pleae check the battery");
    }
    else if(robot_status_.connection_err_){
      stat.summary(2,"Connection error occurred, please check the cable");
    }
    else if( robot_status_.temp_err_ ){
      stat.summary(2,"Motor driver is high temperature, please reboot the robot");
    }
    else if(robot_status_.p_stopped_err_){
      stat.summary(2,"Protective stopped, please release it");
    }
    else if(robot_status_.calib_err_ ){
      stat.summary(2,"Calibration error occurred, please recalibration");
    }
    else if(robot_status_.step_out_err_ ){
      stat.summary(1,"Step-out has occurred");
    }
    else if( robot_status_.res_err_ ){
      stat.summary(1,"Response error has occurred");
    }
    else{
      stat.summary(0,"System all green");
    }

    if(robot_status_.connection_err_ && robot_status_.calib_err_ ){
      stat.summary(2,"E-Stop switch is pushed, please release it");
    }

    stat.add("Communication Error", comm_err_);
    stat.add("Emergency Stopped", robot_status_.connection_err_ && robot_status_.calib_err_ );
    stat.add("Protective Stopped",robot_status_.p_stopped_err_);
    stat.add("Connection Error",robot_status_.connection_err_);
    stat.add("Calibration Error",robot_status_.calib_err_);
    stat.add("Motor Servo OFF",robot_status_.motor_err_);
    stat.add("Temperature Error",robot_status_.temp_err_);
    stat.add("Response Error",robot_status_.res_err_);
    stat.add("Step Out Occurred",robot_status_.step_out_err_);
    stat.add("Power Failed",robot_status_.power_err_);

    robot_info_.robot.level = stat.level;
    robot_info_.robot.status = stat.message;

    if (stat.level == 0 || stat.level == 1)
    {
      robot_info_.driver[0].flag = 0;
      robot_info_.driver[1].flag = 0;
      robot_info_.driver[2].flag = 0;
      robot_info_.driver[3].flag = 0;

      robot_info_.driver[0].status = "Normal";
      robot_info_.driver[1].status = "Normal";
      robot_info_.driver[2].status = "Normal";
      robot_info_.driver[3].status = "Normal";
    }
    else
    {
      if (stat.level != pre_diag_level_ || stat.message != pre_diag_msg_)
      {
        mutex_lower_.lock();
        std::vector<uint16_t> data = controller_lower_->getRobotStatus(0);
        mutex_lower_.unlock();
        for (int i = 0; i < 4; ++i)
        {
          robot_info_.driver[i].flag = data.at(i + 2);
          if (data.at(i + 2) >> 0 & 1)
            robot_info_.driver[i].status = "connection error"; // 0b1 = 0d1
          else if (data.at(i + 2) >> 1 & 1)
            robot_info_.driver[i].status = "calibration error"; // 0b10 = 0d2
          else if (data.at(i + 2) >> 3 & 1)
            robot_info_.driver[i].status = "temperature error"; // 0b1000 = 0d8
          else if (data.at(i + 2) >> 6 & 1)
            robot_info_.driver[i].status = "protective stopped"; // 0b100000 = 0d64
          else if (data.at(i + 2) >> 7 & 1)
            robot_info_.driver[i].status = "power error"; // 0b100000 = 0d128
          else if (data.at(i + 2) >> 2 & 1)
            robot_info_.driver[i].status = "servo off"; // 0b100 = 0d4
          else if (data.at(i + 2) >> 4 & 1)
            robot_info_.driver[i].status = "response error"; // 0b10000 = 0d16
          else if (data.at(i + 2) >> 5 & 1)
            robot_info_.driver[i].status = "step out error"; // 0b100000 = 0d32
        }
      }
    }

    pre_diag_level_ = stat.level;
    pre_diag_msg_ = stat.message;

  }

  bool RobotHW::resetRobotStatusCallback
    (seed_r7_ros_controller::ResetRobotStatus::Request&  _req, 
    seed_r7_ros_controller::ResetRobotStatus::Response& _res)
  {

    ROS_WARN("reset robot status");
    mutex_lower_.lock();
    controller_lower_->getRobotStatus(0xFF);
    mutex_lower_.unlock();

    _res.result = "reset status succeeded";

    return true;
  }

  void RobotHW::cmdVelCallback(const geometry_msgs::TwistConstPtr &_cmd_vel)
  {
    robot_info_.robot.cmd_vel.x = _cmd_vel->linear.x;
    robot_info_.robot.cmd_vel.y = _cmd_vel->linear.y;
    robot_info_.robot.cmd_vel.theta = _cmd_vel->angular.z;
  }

  void RobotHW::odomCallback(const nav_msgs::OdometryConstPtr &_odom)
  {
    robot_info_.robot.odom_vel.x = _odom->twist.twist.linear.x;
    robot_info_.robot.odom_vel.y = _odom->twist.twist.linear.y;
    robot_info_.robot.odom_vel.theta = _odom->twist.twist.angular.z;

    robot_info_.robot.odom_pos.x = _odom->pose.pose.position.x;
    robot_info_.robot.odom_pos.y = _odom->pose.pose.position.y;
    robot_info_.robot.odom_pos.theta = tf::getYaw(_odom->pose.pose.orientation);
  }

}
