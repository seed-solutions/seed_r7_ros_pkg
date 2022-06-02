#include "seed_r7_ros_controller/seed_r7_upper_controller.h"


robot_hardware::UpperController::UpperController(const std::string& _port)
{
  ros::param::get("/joint_settings/upper/joints", name_);
  ros::param::get("/joint_settings/upper/aero_index", aero_index_);
  int raw_data_size, body_data_size, base_data_size;
  ros::param::get("/joint_settings/raw_data_size", raw_data_size);
  ros::param::get("/joint_settings/body_data_size", body_data_size);

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

  raw_data_.resize(raw_data_size);
  fill(raw_data_.begin(),raw_data_.end(),0);

  //make table for remap aero <-> ros
  aero_table_.resize(body_data_size);
  for(size_t i = 0; i < aero_table_.size() ; ++i){
    size_t index = std::distance(aero_index_.begin(), std::find(aero_index_.begin(),aero_index_.end(),i));
    if(index != aero_index_.size()) aero_table_.at(i) = std::make_pair(index,name_.at(index));
  }
}

robot_hardware::UpperController::~UpperController()
{
  if(is_open_) upper_->closePort();
}

void robot_hardware::UpperController::write_1byte(uint16_t _address, uint8_t *_write_data,int write_size){
    upper_->write_1byte(_address, _write_data,write_size);
}

std::vector<uint8_t>  robot_hardware::UpperController::read_1byte(uint16_t _address, int size){
    return upper_->read_1byte(_address,size);
}

void robot_hardware::UpperController::resetting(){
    upper_->resetting();
}


void robot_hardware::UpperController::getPosition()
{
  if(is_open_) raw_data_ = upper_->getPosition(0);

  checkRobotStatus();
}

void robot_hardware::UpperController::sendPosition(uint16_t _time, std::vector<int16_t>& _data)
{
  if(is_open_) raw_data_ = upper_->actuateByPosition(_time, _data.data());
  else raw_data_.assign(_data.begin(), _data.end());

  checkRobotStatus();
}

CosmoCmdReqType robot_hardware::UpperController::getCosmoCmd(){
    return upper_->getCosmoCmd();
}

void robot_hardware::UpperController::sendCosmoCmdResp(CosmoCmdRespType resp)
{
  if(is_open_) upper_->sendCosmoCmdResp(resp);
}

RobotStatusCmdReqType robot_hardware::UpperController::getRobotStatusCmd()
{
  return upper_->getRobotStatusCmd();
}

void robot_hardware::UpperController::sendRobotStatusCmdResp(RobotStatusCmdRespType resp)
{
  if(is_open_) upper_->sendRobotStatusCmdResp(resp);
}

//上半身をvirtualcontrollerで動かすまでは不要
VirtualControllerCmdReqType robot_hardware::UpperController::getVirtualControllerCmd()
{
  return upper_->getVirtualControllerCmd();
}

void robot_hardware::UpperController::remapAeroToRos
(std::vector<int16_t>& _ros, std::vector<int16_t>& _aero)
{
  _ros.resize(name_.size());
  for(size_t i = 0; i < _ros.size(); ++i){
    if(aero_index_.at(i) != -1) _ros.at(i) = _aero.at(aero_index_.at(i));
  }
}


void robot_hardware::UpperController::remapRosToAero
(std::vector<int16_t>& _aero, std::vector<int16_t>& _ros)
{
  _aero.resize(aero_table_.size());
  for(size_t i = 0; i < _aero.size(); ++i){
    _aero.at(i) = _ros.at(aero_table_.at(i).first);
  }
}

void robot_hardware::UpperController::setCurrent(uint8_t _number, uint8_t _max, uint8_t _down) 
{
    if(is_open_)upper_->setCurrent(_number, _max, _down);
}

void robot_hardware::UpperController::runScript(uint8_t _number, uint16_t _script) 
{
    if(is_open_)upper_->runScript(_number, _script);
}

std::string robot_hardware::UpperController::getFirmwareVersion()
{
  if(is_open_) return upper_->getVersion(0);
  else return "";
}

void robot_hardware::UpperController::checkRobotStatus()
{
  if(is_open_) comm_err_ = upper_->comm_err_;
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
