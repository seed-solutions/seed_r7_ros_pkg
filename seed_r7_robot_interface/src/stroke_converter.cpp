#include "stroke_converter.h"

StrokeConverter::StrokeConverter(ros::NodeHandle _nh, std::string _robot_model) :
  nh_(_nh),robot_model_(_robot_model)
{
  file_path_ = ros::package::getPath("seed_r7_robot_interface") + "/config/" + robot_model_.c_str() + "/csv/";

  ROS_INFO("start to make stroke convert table");
  //make table and inverse table
  if(makeTable(shoulder_p.table, "shoulder_p")) makeInvTable(shoulder_p.inv_table,shoulder_p.table);
  if(makeTable(shoulder_r.table, "shoulder_r")) makeInvTable(shoulder_r.inv_table,shoulder_r.table);
  if(makeTable(elbow_p.table, "elbow_p")) makeInvTable(elbow_p.inv_table,elbow_p.table);
  if(makeTable(wrist_p.table, "wrist_p")) makeInvTable(wrist_p.inv_table,wrist_p.table);
  if(makeTable(wrist_r.table, "wrist_r")) makeInvTable(wrist_r.inv_table,wrist_r.table);
  if(makeTable(neck_p.table, "neck_p")) makeInvTable(neck_p.inv_table,neck_p.table);
  if(makeTable(neck_r.table, "neck_r")) makeInvTable(neck_r.inv_table,neck_r.table);
  if(makeTable(waist_p.table, "waist_p")) makeInvTable(waist_p.inv_table,waist_p.table);
  if(makeTable(waist_r.table, "waist_r")) makeInvTable(waist_r.inv_table,waist_r.table);
  if(makeTable(leg.table, "leg")) makeInvTable(leg.inv_table,leg.table);

  ROS_INFO("finish to make stroke convert table");

}

StrokeConverter::~StrokeConverter()
{

}

void StrokeConverter::Angle2Stroke (std::vector<int16_t>& _strokes, const std::vector<double> _angles)
{
  float rad2Deg = 180.0 / M_PI;
  float scale = 100.0;

  if(robot_model_ == "typeF"){
    DiffJoint r_wrist = setDualAngleToStroke(rad2Deg * _angles[23], -rad2Deg * _angles[22], wrist_r.table, wrist_p.table,"pitch");
    DiffJoint l_wrist = setDualAngleToStroke(-rad2Deg * _angles[9], rad2Deg * _angles[8], wrist_r.table, wrist_p.table, "pitch");
    DiffJoint waist = setDualAngleToStroke(-rad2Deg * _angles[2], rad2Deg * _angles[1], waist_r.table, waist_p.table);
    DiffJoint neck = setDualAngleToStroke(rad2Deg * _angles[16], rad2Deg * _angles[15], neck_r.table, neck_p.table);

    _strokes[0] = scale * rad2Deg * _angles[0];
    _strokes[1] = scale * waist.one;
    _strokes[2] = scale * waist.two;

    _strokes[3] = scale * setAngleToStroke(-rad2Deg * _angles[3], shoulder_p.table);
    _strokes[4] = scale * setAngleToStroke(rad2Deg * _angles[4], shoulder_r.table);
    _strokes[5] = -scale * rad2Deg * _angles[5];
    _strokes[6] = scale * setAngleToStroke(180 + rad2Deg * _angles[6], elbow_p.table);
    _strokes[7] = -scale * rad2Deg * _angles[7];
    _strokes[8] = scale * l_wrist.one;
    _strokes[9] = scale * l_wrist.two;
    _strokes[13] = scale * (rad2Deg * _angles[13] + 50.0) * 0.18;

    _strokes[14] = scale * rad2Deg * _angles[14];
    _strokes[15] = scale * neck.two;
    _strokes[16] = scale * neck.one;

    _strokes[17] = scale * setAngleToStroke(-rad2Deg * _angles[17], shoulder_p.table);
    _strokes[18] = scale * setAngleToStroke(-rad2Deg * _angles[18], shoulder_r.table);
    _strokes[19] = -scale * rad2Deg * _angles[19];
    _strokes[20] = scale * setAngleToStroke(180 + rad2Deg * _angles[20], elbow_p.table);
    _strokes[21] = -scale * rad2Deg * _angles[21];
    _strokes[22] = scale * r_wrist.two;
    _strokes[23] = scale * r_wrist.one;
    _strokes[27] = -scale * (rad2Deg * _angles[27] - 50.0) * 0.18;

    _strokes[28] = scale * setAngleToStroke(- rad2Deg * _angles[28], leg.table);  //knee
    _strokes[29] = scale * setAngleToStroke(  rad2Deg * _angles[29], leg.table);  //ankle
  }
  else{
    ROS_ERROR("not supported %s in stroke_converter.cpp", robot_model_.c_str());
  }
}

 //////////////////////////////////////////////////
void StrokeConverter::Stroke2Angle (std::vector<double>& _angles, const std::vector<int16_t> _strokes)
{
  float deg2Rad = M_PI / 180.0;
  float scale = 0.01;

  if(robot_model_ == "typeF"){
    _angles[0] = deg2Rad * scale * _strokes[0];
    _angles[1] = deg2Rad * setStrokeToAngle(scale * (_strokes[2] + _strokes[1]) * 0.5, waist_p.inv_table);
    _angles[2] = deg2Rad * setStrokeToAngle(scale * (_strokes[2] - _strokes[1]) * 0.5, waist_r.inv_table);
    _angles[3] = -deg2Rad * setStrokeToAngle(scale * _strokes[3], shoulder_p.inv_table);
    _angles[4] = deg2Rad * setStrokeToAngle(scale * _strokes[4], shoulder_r.inv_table);
    _angles[5] = -deg2Rad * scale * _strokes[5];
    _angles[6] = - M_PI + deg2Rad * setStrokeToAngle(scale * _strokes[6], elbow_p.inv_table);
    _angles[7] = -deg2Rad * scale * _strokes[7];
    _angles[8] = -deg2Rad * setStrokeToAngle(scale * (_strokes[9] - _strokes[8]) * 0.5, wrist_p.inv_table);
    _angles[9] = -deg2Rad * setStrokeToAngle(scale * (_strokes[9] + _strokes[8]) * 0.5, wrist_r.inv_table);
    _angles[10] = -deg2Rad * (scale * _strokes[13] * 5.556 - 50.0);
    _angles[11] = 0;
    _angles[12] = 0;
    _angles[13] = deg2Rad * (scale * _strokes[13] * 5.556 - 50.0);

    _angles[14] = deg2Rad * scale * _strokes[14];
    _angles[15] = deg2Rad * setStrokeToAngle(scale * (_strokes[16] + _strokes[15]) * 0.5, neck_p.inv_table);
    _angles[16] = -deg2Rad * setStrokeToAngle(scale * (_strokes[16] - _strokes[15]) * 0.5, neck_r.inv_table);

    _angles[17] = -deg2Rad * setStrokeToAngle(scale * _strokes[17], shoulder_p.inv_table);
    _angles[18] = -deg2Rad * setStrokeToAngle(scale * _strokes[18], shoulder_r.inv_table);
    _angles[19] = -deg2Rad * scale * _strokes[19];
    _angles[20] = - M_PI + deg2Rad * setStrokeToAngle(scale * _strokes[20], elbow_p.inv_table);
    _angles[21] = -deg2Rad * scale * _strokes[21];
    _angles[22] = -deg2Rad * setStrokeToAngle(scale * (_strokes[23] - _strokes[22]) * 0.5, wrist_p.inv_table);
    _angles[23] = deg2Rad * setStrokeToAngle(scale * (_strokes[23] + _strokes[22]) * 0.5, wrist_r.inv_table);
    _angles[24] = deg2Rad * (scale * _strokes[27] * 5.556 - 50.0);
    _angles[25] = 0;
    _angles[26] = 0;
    _angles[27] = -deg2Rad * (scale * _strokes[27] * 5.556 - 50.0);

    _angles[28] = -deg2Rad * setStrokeToAngle(scale * _strokes[28], leg.inv_table);  //knee
    _angles[29] = deg2Rad * setStrokeToAngle(scale * _strokes[29], leg.inv_table);  //ankle
  }
  else{
    ROS_ERROR("not supported %s in stroke_converter.cpp", robot_model_.c_str());
  }
}

bool StrokeConverter::makeTable(std::vector<StrokeMap>& _table, std::string _file_name)
{
  std::vector<std::pair<int,float>> csv_data;
  _table.clear();
  std::string str;

  std::ifstream ifs_0(file_path_ + _file_name + ".csv",std::ios_base::in);
  if(!ifs_0.is_open()){
    ROS_ERROR("can't find %s.csv at %s", _file_name.c_str(), file_path_.c_str());
    return false;
  }

  while(getline(ifs_0,str)){
    std::string token;
    std::istringstream stream(str);

    int angle;
    float stroke;

    std::sscanf(str.substr(0,str.find(',')).c_str(),"%d", &angle); //read first column
    std::sscanf(str.substr(str.find_first_of(',')+1).c_str(),"%f", &stroke); //read second column

    csv_data.push_back(std::make_pair(angle,stroke));
  }

  _table.resize(csv_data.size());

  for(size_t i = 0; i < csv_data.size() ; ++i){
    _table.at(i).angle = csv_data.at(i).first;
    _table.at(i).stroke = csv_data.at(i).second;
  }

  //sort in ascending order
  if(_table.at(1).angle < _table.at(0).angle) std::reverse(_table.begin(),_table.end());

  _table.front().range = 0;
  for(size_t i = 1; i < _table.size() ; ++i){
    _table.at(i).range = _table.at(i).stroke - _table.at(i-1).stroke;
  }

  return true;
}

void StrokeConverter::makeInvTable(std::vector<StrokeMap>& _inv_table, std::vector<StrokeMap>& _table){

  int sign = 1;
  _inv_table.clear();
  _inv_table = _table;

  //sort in ascending order
  if(_table.at(1).stroke < _table.at(0).stroke) std::reverse(_inv_table.begin(),_inv_table.end());

  _inv_table.front().range = 0;
  if(_table.at(1).range < 0 ) sign = -1;
  for(size_t i = 1; i < _inv_table.size() ; ++i){
    _inv_table.at(i).range = sign * (_inv_table.at(i).stroke - _inv_table.at(i-1).stroke);
  }
}

float StrokeConverter::setAngleToStroke (float _angle, std::vector<StrokeMap>& _table)
{
  //limit
  if(_angle < _table.front().angle) _angle = _table.front().angle;
  if (_angle > _table.back().angle) _angle = _table.back().angle;

  std::vector<int> angle_vector;
  angle_vector.resize(_table.size());
  for(size_t i=0;i< _table.size();++i) angle_vector.at(i) = _table.at(i).angle;

  // search index less than _stroke;
  size_t index = std::distance(angle_vector.begin(),
        std::upper_bound(angle_vector.begin()+1,angle_vector.end()-1,_angle));

  auto ref = _table.at(index);
  int upper_angle =ref.angle;
  float stroke = ref.stroke;
  float interval = ref.range;

  return stroke - (upper_angle - _angle) * interval;
}

float StrokeConverter::setStrokeToAngle (float _stroke, std::vector<StrokeMap>& _inv_table)
{
  //limit
  if(_stroke < _inv_table.front().stroke) _stroke = _inv_table.front().stroke;
  if (_stroke > _inv_table.back().stroke) _stroke = _inv_table.back().stroke;

  std::vector<float> stroke_vector;
  stroke_vector.resize(_inv_table.size());
  for(size_t i=0;i<_inv_table.size();++i) stroke_vector.at(i) = _inv_table.at(i).stroke;

// search index less than _stroke;
  size_t index = std::distance(stroke_vector.begin(),
      std::upper_bound(stroke_vector.begin()+1,stroke_vector.end()-1,_stroke));

  auto ref = _inv_table.at(index);
  int angle = ref.angle;
  float upper_stroke = ref.stroke;
  float interval = ref.range;

  return angle - (upper_stroke - _stroke) / interval;

}

DiffJoint StrokeConverter::setDualAngleToStroke (float _r_angle, float _p_angle,
    std::vector<StrokeMap>& _r_table, std::vector<StrokeMap>& _p_table, std::string _diff_axis)
{
  float stroke1 = setAngleToStroke(_r_angle,_r_table);
  float stroke2 = setAngleToStroke(_p_angle,_p_table);

  if(_diff_axis == "pitch") return {stroke2 + stroke1, stroke1 - stroke2};
  else return {stroke2 + stroke1, stroke2 - stroke1};

}
