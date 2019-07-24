#include "seed_r7_upper_controller.h"

using namespace noid;
using namespace controller;

NoidUpperController::NoidUpperController(const std::string& _port)
{
  ros::param::get("joint_settings/upper/name",name_);
  ros::param::get("joint_settings/upper/aero_index",aero_index_);
  ros::param::get("joint_settings/upper/ros_index",ros_index_);
  ros::param::get("joint_settings/upper/DOF",DOF_);

  upper_ = new aero::controller::AeroCommand();
  if(upper_->openPort(_port,BAUDRATE)){
    ROS_INFO("%s is connected", _port.c_str());
    upper_->flushPort();
    is_open_ = true;
  }
  else{
    ROS_ERROR("%s is not connected", _port.c_str());
    is_open_ = false;
  }

  raw_data_.resize(31);
  fill(raw_data_.begin(),raw_data_.end(),0);
  
}

NoidUpperController::~NoidUpperController()
{
  if(is_open_) upper_->closePort();
}

void NoidUpperController::getPosition()
{
  if(is_open_) raw_data_ = upper_->getPosition(0);
}


void NoidUpperController::sendPosition(uint16_t _time, std::vector<int16_t>& _data)
{
  if(is_open_) raw_data_ = upper_->actuateByPosition(_time, _data.data());
  else raw_data_.assign(_data.begin(), _data.end());
}

void NoidUpperController::remapAeroToRos(std::vector<int16_t>& _before, std::vector<int16_t>& _after)
{
  for(int i=0; i < DOF_ ; ++i){
    for(size_t j=0; j < ros_index_.size(); ++j){
      if(ros_index_[j] == i){
        _after[i] = _before[aero_index_[j]];
        break;
      }
    }
  }
}


void NoidUpperController::remapRosToAero(std::vector<int16_t>& _before, std::vector<int16_t>& _after)
{
  size_t aero_array_size = 30;
  for(size_t i=0; i < aero_array_size; ++i){
    for(size_t j=0; j < aero_index_.size(); ++j){
      if(aero_index_[j] == i){
        _after[i] = _before[ros_index_[j]];
        break;
      }
    }
  }
}

void NoidUpperController::setCurrent(uint8_t _number, uint8_t _max, uint8_t _down) 
{
    if(is_open_)upper_->setCurrent(_number, _max, _down);
}

void NoidUpperController::runScript(uint8_t _number, uint16_t _script) 
{
    if(is_open_)upper_->runScript(_number, _script);
}
