#include "seed_r7_lower_controller.h"


LowerController::LowerController(const std::string& _port)
{
  ros::param::get("joint_settings/upper/name",upper_name_);
  ros::param::get("joint_settings/lower/name",name_);
  ros::param::get("joint_settings/lower/aero_index",aero_index_);
  ros::param::get("joint_settings/wheel/name",wheel_name_);
  ros::param::get("joint_settings/wheel/aero_index",wheel_aero_index_);

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

  //make table for remap aero <-> ros
  aero_table_.resize(30);
  for(size_t i = 0; i < aero_table_.size() ; ++i){
    size_t index = std::distance(aero_index_.begin(), std::find(aero_index_.begin(),aero_index_.end(),i));
    if(index != aero_index_.size()) aero_table_.at(i) = std::make_pair(index,name_.at(index));
  }

  wheel_table_.resize(30);
 for(size_t i = 0; i < wheel_table_.size() ; ++i){
    size_t index = std::distance(wheel_aero_index_.begin(), std::find(wheel_aero_index_.begin(),wheel_aero_index_.end(),i));
    if(index != wheel_aero_index_.size()) wheel_table_.at(i) = std::make_pair(index,wheel_name_.at(index));
  } 

}

LowerController::~LowerController()
{
  if(is_open_)lower_->closePort();
}

void LowerController::getPosition()
{
  if(is_open_) raw_data_ = lower_->getPosition(0);

}

void LowerController::sendPosition(uint16_t _time, std::vector<int16_t>& _data)
{
  if(is_open_) raw_data_ = lower_->actuateByPosition(_time, _data.data());
  else raw_data_.assign(_data.begin(), _data.end());
}

void LowerController::remapAeroToRos(std::vector<int16_t>& _ros, std::vector<int16_t>& _aero)
{
  _ros.resize(name_.size());
  for(size_t i = 0; i < _ros.size(); ++i){
    if(aero_index_.at(i) != -1) _ros.at(i) = _aero.at(aero_index_.at(i));
  }
}

void LowerController::remapRosToAero(std::vector<int16_t>& _aero, std::vector<int16_t>& _ros)
{
  _aero.resize(aero_table_.size());
  for(size_t i = 0; i < _ros.size(); ++i){
    _aero.at(i) = _ros.at(upper_name_.size() + aero_table_.at(i).first);
  }
}

void LowerController::sendVelocity(std::vector<int16_t>& _data)
{
  std::vector<int16_t> send_data(wheel_table_.size());
  fill(send_data.begin(),send_data.end(),0x7FFF);
  for(size_t i = 0; i < wheel_table_.size(); ++i){
    if(wheel_table_.at(i).second != "") send_data.at(i) = _data.at(wheel_table_.at(i).first);
  } 

  if(is_open_) lower_->actuateBySpeed(send_data.data());
}

void LowerController::onServo(bool _value)
{

  std::vector<uint16_t> data(30);
  fill(data.begin(),data.end(),0x7FFF);

  if(is_open_){
    for(size_t i=0; i< wheel_aero_index_.size() ; ++i){
      lower_->onServo(wheel_aero_index_[i] + 1, _value);
    }
  }


}

