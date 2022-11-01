#include <pluginlib/class_list_macros.h>
#include "stroke_converter.h"

void seed_converter::HA::makeTables()
{
  ROS_INFO("upper's csv_dir is %s", upper_csv_dir.c_str());
  ROS_INFO("lifter's csv_dir is %s", lifter_csv_dir.c_str());

  if (makeTable(shoulder_p.table, upper_csv_dir + "/shoulder_p.csv"))
    makeInvTable(shoulder_p.inv_table, shoulder_p.table);
  if (makeTable(elbow_p.table, upper_csv_dir + "/elbow_p.csv"))
    makeInvTable(elbow_p.inv_table, elbow_p.table);
  if (makeTable(wrist_p.table, upper_csv_dir + "/wrist_p.csv"))
    makeInvTable(wrist_p.inv_table, wrist_p.table);
  if (makeTable(wrist_r.table, upper_csv_dir + "/wrist_r.csv"))
    makeInvTable(wrist_r.inv_table, wrist_r.table);


}

void seed_converter::HA::Angle2Stroke
(std::vector<int16_t>& _strokes, const std::vector<double>& _angles)
{
  static const float rad2Deg = 180.0 / M_PI;
  static const float scale = 100.0;

  _strokes[0] = static_cast<int16_t>(scale * rad2Deg * _angles[0]);
  _strokes[1] = static_cast<int16_t>(scale * rad2Deg * _angles[1]);
  
}

void seed_converter::HA::Stroke2Angle
(std::vector<double>& _angles, const std::vector<int16_t>& _strokes)
{
  static const float deg2Rad = M_PI / 180.0;
  static const float scale_inv = 0.01;

  _angles[0] = deg2Rad * scale_inv * _strokes[0];
  _angles[1] = deg2Rad * scale_inv * _strokes[1];

}

PLUGINLIB_EXPORT_CLASS(seed_converter::HA, seed_converter::StrokeConverter)
