#include "seed_r7_ros_controller/seed_r7_lower_controller.h"


robot_hardware::LowerController::LowerController(const std::string& _port)
{
  ros::param::get("/joint_settings/upper/joints", upper_name_);
  ros::param::get("/joint_settings/lower/joints", name_);
  ros::param::get("/joint_settings/lower/aero_index", aero_index_);
  ros::param::get("/joint_settings/wheel/joints", wheel_name_);
  ros::param::get("/joint_settings/wheel/aero_index", wheel_aero_index_);
  int raw_data_size, body_data_size, base_data_size;
  ros::param::get("/joint_settings/raw_data_size", raw_data_size);
  ros::param::get("/joint_settings/body_data_size", body_data_size);
  ros::param::get("/joint_settings/base_data_size", base_data_size);

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

  raw_data_.resize(raw_data_size);
  fill(raw_data_.begin(),raw_data_.end(),0);

  temp_vol_data_.resize(31);
  fill(temp_vol_data_.begin(), temp_vol_data_.end(), 0);
  status_data_.resize(31);
  fill(status_data_.begin(), status_data_.end(), 0);
  current_data_.resize(31);
  fill(current_data_.begin(), current_data_.end(), 0);
  wheel_angles_.resize(4);
  fill(wheel_angles_.begin(), wheel_angles_.end(), 0);

  //make table for remap aero <-> ros
  aero_table_.resize(body_data_size);
  for(size_t i = 0; i < aero_table_.size() ; ++i){
    size_t index = std::distance(aero_index_.begin(), std::find(aero_index_.begin(),aero_index_.end(),i));
    if(index != aero_index_.size()) aero_table_.at(i) = std::make_pair(index,name_.at(index));
  }

  wheel_table_.resize(base_data_size);
  for(size_t i = 0; i < wheel_table_.size() ; ++i){
    size_t index = std::distance(wheel_aero_index_.begin(), std::find(wheel_aero_index_.begin(),wheel_aero_index_.end(),i));
    if(index != wheel_aero_index_.size()) wheel_table_.at(i) = std::make_pair(index,wheel_name_.at(index));
  }
}

robot_hardware::LowerController::~LowerController()
{
  if(is_open_)lower_->closePort();
}

void robot_hardware::LowerController::getPosition()
{
  if (is_open_)
  {
    raw_data_ = lower_->getPosition(0);
    for (size_t i = 0; i < wheel_angles_.size(); ++i)
    {
      wheel_angles_.at(wheel_aero_index_.at(i) - 2) = raw_data_.at(wheel_aero_index_.at(i)) * (M_PI/ 180.0);
    }
  }

  checkRobotStatus();
}

void robot_hardware::LowerController::sendPosition(uint16_t _time, std::vector<int16_t>& _data)
{
  if (is_open_)
  {
    raw_data_ = lower_->actuateByPosition(_time, _data.data());
    for (size_t i = 0; i < wheel_angles_.size(); ++i)
    {
      wheel_angles_.at(wheel_aero_index_.at(i) - 2) = raw_data_.at(wheel_aero_index_.at(i)) * (M_PI/ 180.0);
    }
  }
  else
    raw_data_.assign(_data.begin(), _data.end());

  checkRobotStatus();
}

void robot_hardware::LowerController::remapAeroToRos
(std::vector<int16_t>& _ros, std::vector<int16_t>& _aero)
{
  _ros.resize(name_.size());
  for(size_t i = 0; i < _ros.size(); ++i){
    if(aero_index_.at(i) != -1) _ros.at(i) = _aero.at(aero_index_.at(i));
  }
}

void robot_hardware::LowerController::remapRosToAero
(std::vector<int16_t>& _aero, std::vector<int16_t>& _ros)
{
  _aero.resize(aero_table_.size());
  for(size_t i = 0; i < _aero.size(); ++i){
    _aero.at(i) = _ros.at(upper_name_.size() + aero_table_.at(i).first);
  }
}

void robot_hardware::LowerController::runScript(uint8_t _number, uint16_t _script) 
{
    if(is_open_)lower_->runScript(_number, _script);
}
void robot_hardware::LowerController::sendVelocity(std::vector<int16_t>& _data)
{
  std::vector<int16_t> send_data(wheel_table_.size());
  fill(send_data.begin(),send_data.end(),0x7FFF);
  for(size_t i = 0; i < wheel_table_.size(); ++i){
    if(wheel_table_.at(i).second != "") send_data.at(i) = _data.at(wheel_table_.at(i).first);
  } 

  if(is_open_) lower_->actuateBySpeed(send_data.data());
}

void robot_hardware::LowerController::onServo(bool _value)
{
  std::vector<uint16_t> data(30);
  fill(data.begin(),data.end(),0x7FFF);

  if(is_open_){
    for(size_t i=0; i< wheel_aero_index_.size() ; ++i){
      lower_->onServo(wheel_aero_index_[i] + 1, _value);
      //reset present position
      lower_->throughCAN(wheel_aero_index_[i] + 1, 0x6F,0,1,0,0,0);
    }
  }
}

void robot_hardware::LowerController::stopPolling()
{
  if (is_open_)
  {
    lower_->onServo(0, 3);
    usleep(100 * 1000); // magic number
  }
}

std::vector<uint16_t> robot_hardware::LowerController::getBatteryVoltage()
{
  if (is_open_)
  {
    temp_vol_data_ = lower_->getTemperatureVoltage(0);
    // return lower_->getTemperatureVoltage(31)[0] * 0.1;
  }

  return temp_vol_data_;
}

std::vector<uint16_t> robot_hardware::LowerController::getMotorCurrent(int8_t _number)
{
  if (is_open_)
  {
    current_data_ = lower_->getCurrent(_number);
  }

  return current_data_;
}

std::string robot_hardware::LowerController::getFirmwareVersion(uint8_t _number)
{
  if (is_open_)
    return lower_->getVersion(_number);
  else
    return "";
}

std::vector<uint16_t> robot_hardware::LowerController::getRobotStatus(int8_t _number)
{
  if (is_open_ && _number == 0xFF)
  {
    lower_->getStatus(_number);
    //_number == 0xFF : reset flag
  }
  else if (is_open_)
  {
    status_data_ = lower_->getStatus(_number);
  }
  return status_data_;
}


void robot_hardware::LowerController::checkRobotStatus()
{
  if(is_open_) comm_err_ = lower_->comm_err_;
  else comm_err_ = false;

  int16_t status_bit = raw_data_.back();
  int8_t max_bit_size = 16;

  if(!is_open_) status_bit = 0;
  robot_status_.connection_err_ = (status_bit >> error_bit_t::can1_connection)&1 ||
    (status_bit >> error_bit_t::can2_connection)&1;

  robot_status_.calib_err_ = (status_bit >> error_bit_t::can1_calibration)&1 ||
    (status_bit >> error_bit_t::can2_calibration)&1;

  robot_status_.motor_err_ = (status_bit >> error_bit_t::can1_motor_status)&1 ||
    (status_bit >> error_bit_t::can2_motor_status)&1;

  robot_status_.temp_err_ = (status_bit >> error_bit_t::can1_temperature)&1 ||
    (status_bit >> error_bit_t::can2_temperature)&1;

  robot_status_.res_err_ = (status_bit >> error_bit_t::can1_response)&1 ||
    (status_bit >> error_bit_t::can2_response)&1;

  robot_status_.step_out_err_ = (status_bit >> error_bit_t::can1_step_out)&1 ||
    (status_bit >> error_bit_t::can2_step_out)&1;

  robot_status_.p_stopped_err_ = (status_bit >> error_bit_t::can1_protective_stopped)&1 ||
    (status_bit >> error_bit_t::can2_protective_stopped)&1;

  robot_status_.power_err_ = (status_bit >> error_bit_t::can1_power)&1 ||
    (status_bit >> error_bit_t::can2_power)&1;

}
