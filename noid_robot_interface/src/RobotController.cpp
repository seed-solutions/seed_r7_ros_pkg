#include "RobotController.hh"

using namespace noid;
using namespace controller;

NoidControllerProto::NoidController(const std:string& _port, uint8_t _id) 
{
}

NoidControllerProto::~NoidController()
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

  // if (cmd == CMD_MOVE_ABS || cmd == CMD_WATCH_MISSTEP || cmd == CMD_GET_POS) {
  if (cmd == CMD_WATCH_MISSTEP) {
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

bool NoidControllerProto::get_joint_name(int32_t _joint_id, std::string &_name);
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

//////////////////////////////////////////////////
int16_t decode_short_(uint8_t* _raw)
{
  int16_t value;
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&value);
  bvalue[0] = _raw[1];
  bvalue[1] = _raw[0];
  return value;
}


/* ---------   NoidUpperController ----------------*/
NoidUpperController::NoidUpperController(const std::string& _port) :
    NoidControllerProto(_port, ID_UPPER)
{
    stroke_cur_vector_.resize(AERO_DOF_UPPER);

    stroke_joint_indices_.clear();
    stroke_joint_indices_.reserve(AERO_DOF_UPPER);
    
    // This function load angle_joint_indices definition(joint_state_name)
    // generated from Constants.hh
    //AngleJointIndiceUpper();

    get_command(0x41, stroke_cur_vector_);
}

NoidUpperController::~NoidUpperController()
{

}

NoidLowerController::NoidLowerController(const std::string& _port) :
    NoidControllerProto(_port, ID_LOWER)
{
    stroke_cur_vector_.resize(AERO_DOF_LOWER);
    stroke_joint_indices_.clear();
    stroke_joint_indices_.reserve(AERO_DOF_LOWER);

    //AngleJointIndicesLower();

    get_command(0x41, stroke_cur_vector_);

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




std::string NoidLowerController::get_wheel_name(size_t _idx)
{
  return wheel_indices_[_idx].joint_name;
}

std::vector<int16_t>& NoidLowerController::get_reference_wheel_vector()
{
  return wheel_ref_vector_;
}

int32_t NoidsLowerController::get_wheel_id(std::string& _name)
{
  for (size_t i = 0; i < wheel_indices_.size(); ++i) {
    if (wheel_indices_[i].joint_name == _name)
      return static_cast<int32_t>(wheel_indices_[i].stroke_index);
  }
  return -1;
}



