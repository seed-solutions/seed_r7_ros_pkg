#ifndef _STROKE_CONVERTER_H_
#define _STROKE_CONVERTER_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>

struct StrokeMap
{
  int angle;
  float stroke;
  float range;
};

struct ConvertTable
{
  std::vector<StrokeMap> table;
  std::vector<StrokeMap> inv_table;
};

struct DiffJoint
{
  float one;
  float two;
};

class StrokeConverter
{
public:
  StrokeConverter(ros::NodeHandle _nh, std::string _robot_model);
  ~StrokeConverter();

  void Angle2Stroke(std::vector<int16_t>& _strokes, const std::vector<double> _angles);
  void Stroke2Angle(std::vector<double>& _angles, const std::vector<int16_t> _strokes);
  bool makeTable(std::vector<StrokeMap>& _table, std::string _file_name);
  void makeInvTable(std::vector<StrokeMap>& _inv_table,std::vector<StrokeMap>& _table);
  float setAngleToStroke (float _angle, std::vector<StrokeMap>& _table);
  float setStrokeToAngle (float _stroke, std::vector<StrokeMap>& _inv_table);

  DiffJoint setDualAngleToStroke (float _r_angle, float _p_angle,
      std::vector<StrokeMap>& _r_table, std::vector<StrokeMap>& _p_table, std::string _diff_axis="roll");


  ConvertTable shoulder_p,shoulder_r,elbow_p,wrist_p,wrist_r,neck_p,neck_r,waist_p,waist_r,leg;

private:
  ros::NodeHandle nh_;
  std::string file_path_;
  std::string robot_model_;


};

#endif




