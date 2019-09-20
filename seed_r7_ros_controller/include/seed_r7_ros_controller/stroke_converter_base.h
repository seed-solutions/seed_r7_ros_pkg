#ifndef _STROKE_CONVERTER_H_
#define _STROKE_CONVERTER_H_

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>


namespace seed_converter
{

struct StrokeMap
{
  float angle;
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
  StrokeConverter();
  ~StrokeConverter();

  bool initialize(ros::NodeHandle& _nh);
  virtual void makeTables()=0;
  virtual void Angle2Stroke(std::vector<int16_t>& _strokes, const std::vector<double>& _angles)=0;
  virtual void Stroke2Angle(std::vector<double>& _angles, const std::vector<int16_t>& _strokes)=0;

protected:
  bool makeTable(std::vector<StrokeMap>& _table, const std::string _file_name);
  void makeInvTable(std::vector<StrokeMap>& _inv_table, const std::vector<StrokeMap>& _table);
  float setAngleToStroke(const float _angle, const std::vector<StrokeMap>& _table);
  float setStrokeToAngle(const float _stroke, const std::vector<StrokeMap>& _inv_table);

  DiffJoint setDualAngleToStroke
  (const float _r_angle, const float _p_angle,
   const std::vector<StrokeMap>& _r_table, const std::vector<StrokeMap>& _p_table,
   const bool _is_pitch=false);

  std::string file_path_;
};

}

#endif
