#include "noid_lower_controller.h"

using namespace noid;
using namespace controller;

NoidLowerController::NoidLowerController(const std::string& _port)
{
  ros::param::get("joint_settings/lower/name",name_);
  ros::param::get("joint_settings/lower/aero_index",aero_index_);
  ros::param::get("joint_settings/lower/ros_index",ros_index_);
  ros::param::get("joint_settings/lower/DOF",DOF_);
  ros::param::get("joint_settings/wheel/aero_index",wheel_aero_index_);
  ros::param::get("joint_settings/wheel/ros_index",wheel_ros_index_);

  lower_ = new aero::controller::AeroCommand();
  if(lower_->openPort(_port,BAUDRATE)){
    ROS_INFO("%s is connected", _port.c_str());
    lower_->flushPort();
    is_open_ = true;
  }
  else{
    ROS_ERROR("%s is not connected", _port.c_str());
    is_open_ = false;
  }

  raw_data_.resize(31);
  fill(raw_data_.begin(),raw_data_.end(),0);

}

NoidLowerController::~NoidLowerController()
{
  if(is_open_)lower_->closePort();
}

void NoidLowerController::getPosition()
{
  if(is_open_) raw_data_ = lower_->getPosition(0);

}

void NoidLowerController::sendPosition(uint16_t _time, std::vector<int16_t>& _data)
{
  if(is_open_) raw_data_ = lower_->actuateByPosition(_time, _data.data());
  else raw_data_.assign(_data.begin(), _data.end());
}

void NoidLowerController::remapAeroToRos(std::vector<int16_t>& _before, std::vector<int16_t>& _after)
{
  for(int i=0; i < DOF_; ++i){
    for(size_t j=0; j < ros_index_.size(); ++j){
      if(ros_index_[j] == i){
        _after[i] = _before[aero_index_[j]];
        break;
      }
    }
  }
}

void NoidLowerController::remapRosToAero(std::vector<int16_t>& _before, std::vector<int16_t>& _after)
{
  size_t aero_array_size = 30;
  for(size_t i=0; i < aero_array_size; ++i){
    for(size_t j=0; j < aero_index_.size(); ++j){
      if(aero_index_[j] == i){
        _after[i] = _before[ros_index_[j] + (_before.size()-DOF_)];
        break;
      }
    }
  }
}

void NoidLowerController::sendVelocity(std::vector<int16_t>& _data)
{
  size_t aero_array_size = 30;
  std::vector<int16_t> send_data(aero_array_size);
  fill(send_data.begin(),send_data.end(),0x7FFF);

  for(size_t i=0; i < aero_array_size; ++i){
    for(size_t j=0; j < wheel_aero_index_.size(); ++j){
      if(wheel_aero_index_[j] == i){
        send_data[i] = _data[wheel_ros_index_[j]];
        break;
      }
    }
  }

  if(is_open_) lower_->actuateBySpeed(send_data.data());
}

void NoidLowerController::onServo(bool _value)
{

  std::vector<uint16_t> data(30);
  fill(data.begin(),data.end(),0x7FFF);

  if(is_open_){
    for(size_t i=0; i< wheel_aero_index_.size() ; ++i){
      lower_->onServo(wheel_aero_index_[i] + 1, _value);
    }
  }


}

