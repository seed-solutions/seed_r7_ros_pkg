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

 //same as elecom controller
   joy_.axes.resize(6);
   fill(joy_.axes.begin(),joy_.axes.end(),0);
   joy_.buttons.resize(13);
   fill(joy_.buttons.begin(),joy_.buttons.end(),0);
   enable_joy_ = false;
}

robot_hardware::LowerController::~LowerController()
{
  if(is_open_)lower_->closePort();
}

void robot_hardware::LowerController::write_1byte(uint16_t _address, uint8_t *_write_data, int write_size) {
    lower_->write_1byte(_address, _write_data, write_size);
}


std::vector<uint8_t>  robot_hardware::LowerController::read_1byte(uint16_t _address,int size){
    return lower_->read_1byte(_address,size);
}

void robot_hardware::LowerController::resetting(){
    lower_->resetting();
}


void robot_hardware::LowerController::getPosition()
{
  if(is_open_) raw_data_ = lower_->getPosition(0);

  checkRobotStatus();
}

void robot_hardware::LowerController::sendPosition(uint16_t _time, std::vector<int16_t>& _data)
{
	if(is_open_){
		raw_data_ = lower_->actuateByPosition(_time, _data.data());
	}
	else raw_data_.assign(_data.begin(), _data.end());

  checkRobotStatus();
}

void robot_hardware::LowerController::setJoy(std::vector<uint8_t>& _data)
{
  uint8_t BTL = _data[4];
  uint8_t BTR = _data[5];
  uint8_t L_LR = _data[6];
  uint8_t L_UD = _data[7];
  uint8_t R_LR = _data[8];
  uint8_t R_UD = _data[9];

  ROS_INFO("joy: %d,%d,%d,%d,%d,%d", BTL,BTR,L_LR,L_UD,R_LR,R_UD);

  joy_.buttons[0] = (BTR >> 7) & 1; //□ = X
  joy_.buttons[1] = (BTR >> 4) & 1; //△ = Y
  joy_.buttons[2] = (BTR >> 6) & 1; //× = A
  joy_.buttons[3] = (BTR >> 5) & 1; //○ = B

  joy_.buttons[4] = (BTR >> 2) & 1; //L1 = LB
  joy_.buttons[5] = (BTR >> 3) & 1; //R1 = RB
  joy_.buttons[6] = (BTR >> 0) & 1; //L2 = LT
  joy_.buttons[7] = (BTR >> 1) & 1; //R2 = RT

  joy_.buttons[8] = (BTL >> 0) & 1; //Analog L push
  joy_.buttons[9] = (BTL >> 2) & 1; //Analog R push
  joy_.buttons[10] = (BTL >> 1) & 1; //select
  joy_.buttons[11] = (BTL >> 3) & 1; //start

  joy_.axes[0] = static_cast<float>(L_LR - 127)/127;  //left joystick
  joy_.axes[1] = static_cast<float>(127 - L_UD)/127;  //left joystick
  joy_.axes[2] = static_cast<float>(127 - R_UD)/127;  //right joystick
  joy_.axes[3] = static_cast<float>(R_LR - 127)/127;  //right joystick

  if((BTL >> 7) & 1) joy_.axes[4] = 1;          //←
  else if((BTL >> 5) & 1 ) joy_.axes[4] = -1;   //→
  else joy_.axes[4] = 0;

  if((BTL >> 4) & 1) joy_.axes[5] = 1;          //←
  else if((BTL >> 6) & 1 ) joy_.axes[5] = -1;   //→
  else joy_.axes[5] = 0;

  joy_.header.stamp = ros::Time::now();
  joy_.header.frame_id = "/cosmo/joy";
}

CosmoCmdReqType robot_hardware::LowerController::getCosmoCmd(){
    return lower_->getCosmoCmd();
}

void robot_hardware::LowerController::sendCosmoCmdResp(CosmoCmdRespType resp)
{
  if(is_open_) lower_->sendCosmoCmdResp(resp);
}

RobotStatusCmdReqType robot_hardware::LowerController::getRobotStatusCmd()
{
  return lower_->getRobotStatusCmd();
}

void robot_hardware::LowerController::sendRobotStatusCmdResp(RobotStatusCmdRespType resp)
{
  if(is_open_) lower_->sendRobotStatusCmdResp(resp);
}

VirtualControllerCmdReqType robot_hardware::LowerController::getVirtualControllerCmd()
{
    return lower_->getVirtualControllerCmd();
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

float robot_hardware::LowerController::getBatteryVoltage()
{
  if(is_open_) return lower_->getTemperatureVoltage(31)[0] * 0.1;
  else return 0;
}

std::string robot_hardware::LowerController::getFirmwareVersion()
{
  if(is_open_) return lower_->getVersion(0);
  else return "";
}

void robot_hardware::LowerController::getRobotStatus(int8_t _number)
{
  if(is_open_)lower_->getStatus(_number);

  //_number == 0xFF : reset flag
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
