#include "RobotController.h"
#include "ros/ros.h"

using namespace noid;
using namespace controller;

NoidControllerProto::NoidControllerProto(const std::string& _port, uint8_t _id) 
{
}

NoidControllerProto::~NoidControllerProto()
{
}


void NoidControllerProto::get_data(std::vector<int16_t>& _stroke_vector)
{
  std::vector<uint8_t> dat;
  dat.resize(RAW_DATA_LENGTH);
   
  //include seed_solutions_sdk
  //seed_.read(dat);

  uint16_t header = decode_short_(&dat[0]);
  int16_t cmd;
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&cmd);
  bvalue[0] = dat[3];
  bvalue[1] = 0x00;

  if (header != 0xdffd) {
    //include seed_solutions_sdk
    //seed_.flush();
    std::cerr << "Proto: ERROR: invalid header" << std::endl;
    return;
  }

  if (cmd == 0x11 ||  // TMOVE
      cmd == 0x14 ||  // MOVE
      cmd == 0x41 ||  
      cmd == 0x42 ||
      cmd == 0x43 ||
      cmd == 0x44 ||
      cmd == 0x45 ||
      cmd == 0x52
     ) 
  {
    _stroke_vector.resize(stroke_joint_indices_.size());
    // raw to stroke
    for (size_t i = 0; i < stroke_joint_indices_.size(); ++i) {
      AJointIndex& aji = stroke_joint_indices_[i];
      // uint8_t -> uint16_t
      _stroke_vector[aji.stroke_index] =
        decode_short_(&dat[RAW_HEADER_OFFSET + aji.raw_index * 2]);

      // check value
      if (_stroke_vector[aji.stroke_index] > 0x7fff) {
        _stroke_vector[aji.stroke_index] -= std::pow(2, 16);
      } else if (_stroke_vector[aji.stroke_index] == 0x7fff) {
        _stroke_vector[aji.stroke_index] = 0;
      }
    }
  }

  if (cmd == 0x52) {
    uint8_t status0 = dat[RAW_HEADER_OFFSET + 60];
    uint8_t status1 = dat[RAW_HEADER_OFFSET + 61];
    if ((status0 >> 5) == 1 || (status1 >> 5) == 1) {
      bad_status_ = true;
    } else {
      bad_status_ = false;
    }
  }

}

void NoidControllerProto::get_command(uint8_t _cmd, uint8_t _sub,
                                      std::vector<int16_t>& _stroke_vector)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  std::vector<uint8_t> dat(RAW_DATA_LENGTH);
  //include seed_solutions_sdk
  //seed_.send_command(_cmd, _sub, 0, dat);
  get_data(_stroke_vector);
}

void NoidControllerProto::get_command(uint8_t _cmd,
                                      std::vector<int16_t>& _stroke_vector)
{
  get_command(_cmd, 0x00, _stroke_vector);
}

std::vector<int16_t> NoidControllerProto::get_actual_stroke_vector()
{
    return stroke_cur_vector_;
}

bool NoidControllerProto::get_status()
{
  return bad_status_;
}

int NoidControllerProto::get_number_of_angle_joints()
{
    return angle_joint_indices_.size();
}


std::string NoidControllerProto::get_stroke_joint_name(size_t _idx)
{
    return stroke_joint_indices_[_idx].joint_name;
}

int NoidControllerProto::get_number_of_strokes()
{
    return stroke_joint_indices_.size();
}

bool NoidControllerProto::get_joint_name(int32_t _joint_id, std::string &_name)
{
  for (auto it = angle_joint_indices_.begin();
       it != angle_joint_indices_.end(); it++ ) 
  {
    if (it->second == _joint_id) 
    {
      _name = it->first;
      return true;
    }
  } 
}

void NoidControllerProto::set_position(
    std::vector<int16_t>& _stroke_vector, uint16_t _time)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  // for ROS
  ROS_INFO("strokes_vector:%d", _stroke_vector.size());
  ROS_INFO("strokes_ref_vector:%d", stroke_ref_vector_.size());
  ROS_INFO("strokes_cur_vector:%d", stroke_cur_vector_.size());



  for (size_t i = 0; i < _stroke_vector.size(); ++i) {
    if (_stroke_vector[i] != 0x7fff) {
      stroke_ref_vector_[i] = _stroke_vector[i];
    }
  }

  // for seed
  std::vector<uint8_t> dat(RAW_DATA_LENGTH);
  stroke_to_raw_(_stroke_vector, dat);
  //seed_.flush();

  //include seed_solutions_sdk
  //seed_.send_command(CMD_MOVE_ABS_POS_RET, _time, dat);

  // for ROS

  get_data(stroke_cur_vector_);
  
}

void NoidControllerProto::stroke_to_raw_(std::vector<int16_t>& _stroke,
                                         std::vector<uint8_t>& _raw)
{
  for (size_t i = 0; i < stroke_joint_indices_.size(); ++i) {
    AJointIndex& aji = stroke_joint_indices_[i];
    // uint16_t -> uint8_t
    encode_short_(_stroke[aji.stroke_index],
                  &_raw[RAW_HEADER_OFFSET + aji.raw_index * 2]);
  }
}





/* ---------   NoidUpperController ----------------*/
NoidUpperController::NoidUpperController(const std::string& _port) :
    NoidControllerProto(_port, ID_UPPER)
{
    stroke_cur_vector_.resize(AERO_DOF_UPPER);
    stroke_ref_vector_.resize(AERO_DOF_UPPER);


    stroke_joint_indices_.clear();
    stroke_joint_indices_.reserve(AERO_DOF_UPPER);
    
    // This function load angle_joint_indices definition(joint_state_name)
    // generated from Constants.hh
    angle_joint_indices_["waist_y_joint"] = 0;
    angle_joint_indices_["waist_p_joint"] = 1;
    angle_joint_indices_["waist_r_joint"] = 2;
    angle_joint_indices_["l_shoulder_p_joint"] = 3;
    angle_joint_indices_["l_shoulder_r_joint"] = 4;
    angle_joint_indices_["l_shoulder_y_joint"] = 5;
    angle_joint_indices_["l_elbow_joint"] = 6;
    angle_joint_indices_["l_wrist_y_joint"] = 7;
    angle_joint_indices_["l_wrist_p_joint"] = 8;
    angle_joint_indices_["l_wrist_r_joint"] = 9;
    angle_joint_indices_["l_indexbase_joint"] = 10;
    angle_joint_indices_["l_indexmid_joint"] = 11;
    angle_joint_indices_["l_indexend_joint"] = 12;
    angle_joint_indices_["l_thumb_joint"] = 13;
    angle_joint_indices_["neck_y_joint"] = 14;
    angle_joint_indices_["neck_p_joint"] = 15;
    angle_joint_indices_["neck_r_joint"] = 16;
    angle_joint_indices_["r_shoulder_p_joint"] = 17;
    angle_joint_indices_["r_shoulder_r_joint"] = 18;
    angle_joint_indices_["r_shoulder_y_joint"] = 19;
    angle_joint_indices_["r_elbow_joint"] = 20;
    angle_joint_indices_["r_wrist_y_joint"] = 21;
    angle_joint_indices_["r_wrist_p_joint"] = 22;
    angle_joint_indices_["r_wrist_r_joint"] = 23;
    angle_joint_indices_["r_indexbase_joint"] = 24;
    angle_joint_indices_["r_indexmid_joint"] = 25;
    angle_joint_indices_["r_indexend_joint"] = 26;
    angle_joint_indices_["r_thumb_joint"] = 27;

    get_command(0x41, stroke_cur_vector_);
    stroke_ref_vector_.assign(stroke_cur_vector_.begin(),
                            stroke_cur_vector_.end());
}

NoidUpperController::~NoidUpperController()
{

}

NoidLowerController::NoidLowerController(const std::string& _port) :
    NoidControllerProto(_port, ID_LOWER)
{
    stroke_cur_vector_.resize(AERO_DOF_LOWER);
    stroke_ref_vector_.resize(AERO_DOF_LOWER);

    stroke_joint_indices_.clear();
    stroke_joint_indices_.reserve(AERO_DOF_LOWER);

    //AngleJointIndicesLower();
    angle_joint_indices_["knee_joint"] = 28;
    angle_joint_indices_["ankle_joint"] = 29;

    get_command(0x41, stroke_cur_vector_);
    stroke_ref_vector_.assign(stroke_cur_vector_.begin(),
                            stroke_cur_vector_.end());
}

NoidLowerController::~NoidLowerController()
{
    
}

void NoidLowerController::servo_command(int16_t _d0, int16_t _d1)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  std::vector<int16_t> stroke_vector(stroke_joint_indices_.size(), _d0);
  std::vector<uint8_t> dat(RAW_DATA_LENGTH);

  stroke_to_raw_(stroke_vector, dat);

  // adding code
  encode_short_(_d1, &dat[RAW_HEADER_OFFSET + 2 * 2]);
  encode_short_(_d1, &dat[RAW_HEADER_OFFSET + 3 * 2]);
  encode_short_(_d1, &dat[RAW_HEADER_OFFSET + 4 * 2]);
  encode_short_(_d1, &dat[RAW_HEADER_OFFSET + 5 * 2]);

   //include seed_solutios_sdk
  //seed_.send_command(CMD_MOTOR_SRV, 0, dat);
}

void NoidLowerController::set_wheel_velocity(
    std::vector<int16_t>& _wheel_vector, uint16_t _time)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  std::vector<uint8_t> dat(RAW_DATA_LENGTH);

  // use previous reference strokes to keep its positions
  stroke_to_raw_(stroke_ref_vector_, dat);

  // wheel to raw
  for (size_t i = 0; i < wheel_indices_.size(); ++i) {
    AJointIndex& aji = wheel_indices_[i];
    encode_short_(_wheel_vector[aji.stroke_index],
                  &dat[RAW_HEADER_OFFSET + aji.raw_index * 2]);
  }
  //include seed_solutions_sdk
  //seed_.send_command(CMD_MOVE_SPD, _time, dat);

  // MoveAbs returns current stroke
  // MOVE_SPD also returns data, it is different behavior from Aero Command List ???
  std::vector<uint8_t> dummy;
  dummy.resize(RAW_DATA_LENGTH);
  //seed_solutions_sdk
  //seed_.read(dummy);
}
#if 0
void NoidLowerController::VelocityToWheel(double _linear_x, double _linear_y, double _angular_z,std::vector<int16_t>& _wheel_vel) 
{
  float dx, dy, dtheta, theta;
  float v1, v2, v3, v4;
  int16_t FR_wheel, RR_wheel, FL_wheel, RL_wheel;
  heta = 0.0;  // this means angle in local coords, so always 0

  float cos_theta = cos(theta);
  float sin_theta = sin(theta);

  // change dy and dx, because of between ROS and vehicle direction
  dy = (_linear_x * cos_theta - _linear_y * sin_theta);
  dx = (_linear_x * sin_theta + _linear_y * cos_theta);
  dtheta = _angular_z;  // desirede angular velocity

  // calculate wheel velocity
  v1 = ktheta * dtheta +
    kv * ((-cos_theta + sin_theta) * dx + (-cos_theta - sin_theta) * dy);
  v2 = ktheta * dtheta +
    kv * ((-cos_theta - sin_theta) * dx + ( cos_theta - sin_theta) * dy);
  v3 = ktheta * dtheta +
    kv * (( cos_theta - sin_theta) * dx + ( cos_theta + sin_theta) * dy);
  v4 = ktheta * dtheta +
    kv * (( cos_theta + sin_theta) * dx + (-cos_theta + sin_theta) * dy);

  //[rad/sec] -> [deg/sec]
  FR_wheel = static_cast<int16_t>(v1 * (180 / M_PI));
  RR_wheel = static_cast<int16_t>(v4 * (180 / M_PI));
  FL_wheel = static_cast<int16_t>(v2 * (180 / M_PI));
  RL_wheel = static_cast<int16_t>(v3 * (180 / M_PI));

  _wheel_vel[0] = FL_wheel;
  _wheel_vel[1] = FR_wheel;
  _wheel_vel[2] = RL_wheel;
  _wheel_vel[3] = RR_wheel;
  }
#endif




std::string NoidLowerController::get_wheel_name(size_t _idx)
{
  return wheel_indices_[_idx].joint_name;
}

std::vector<int16_t>& NoidLowerController::get_reference_wheel_vector()
{
  return wheel_ref_vector_;
}

int32_t NoidLowerController::get_wheel_id(std::string& _name)
{
  for (size_t i = 0; i < wheel_indices_.size(); ++i) {
    if (wheel_indices_[i].joint_name == _name)
      return static_cast<int32_t>(wheel_indices_[i].stroke_index);
  }
  return -1;
}

void NoidLowerController::startWheelServo() {
  //ROS_DEBUG("servo on");

  mutex_lower_.lock();
  servo_command(0x7fff, 1);
  mutex_lower_.unlock();
}

void NoidLowerController::stopWheelServo() {
  //ROS_DEBUG("servo off");

  mutex_lower_.lock();
  servo_command(0x7fff, 0);
  mutex_lower_.unlock();
}

//////////////////////////////////////////////////
int16_t noid::controller::decode_short_(uint8_t* _raw)
{
  int16_t value;
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&value);
  bvalue[0] = _raw[1];
  bvalue[1] = _raw[0];
  return value;
}

//////////////////////////////////////////////////
void noid::controller::encode_short_(int16_t _value, uint8_t* _raw)
{
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&_value);
  _raw[0] = bvalue[1];
  _raw[1] = bvalue[0];
}



