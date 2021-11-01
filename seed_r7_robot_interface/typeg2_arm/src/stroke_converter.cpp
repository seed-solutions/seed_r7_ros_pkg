#include <pluginlib/class_list_macros.h>
#include "stroke_converter.h"

void seed_converter::TypeG2Arm::makeTables()
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

  if (makeTable(leg.table, lifter_csv_dir + "/leg.csv"))
    makeInvTable(leg.inv_table, leg.table);
}

void seed_converter::TypeG2Arm::Angle2Stroke
(std::vector<int16_t>& _strokes, const std::vector<double>& _angles)
{
  static const float rad2Deg = 180.0 / M_PI;
  static const float scale = 100.0;

  seed_converter::DiffJoint wrist
    = setDualAngleToStroke(rad2Deg * _angles[5], rad2Deg * _angles[4],
                           wrist_r.table, wrist_p.table, false);

  _strokes[0] = static_cast<int16_t>(scale * rad2Deg * _angles[0]);
  _strokes[1]
    = static_cast<int16_t>(scale * setAngleToStroke(rad2Deg * _angles[1], shoulder_p.table));
  _strokes[2]
    = static_cast<int16_t>(scale * setAngleToStroke(-90 + rad2Deg * _angles[2], elbow_p.table));
  _strokes[3] = static_cast<int16_t>(scale * rad2Deg * _angles[3]);
  _strokes[4] = static_cast<int16_t>(scale * wrist.two);
  _strokes[5] = static_cast<int16_t>(scale * wrist.one);
  _strokes[6] = static_cast<int16_t>(scale * (-rad2Deg * _angles[6] + 50.0) * 0.18);

  _strokes[7]
    = static_cast<int16_t>(scale * setAngleToStroke(- rad2Deg * _angles[7], leg.table));  //knee
  _strokes[8]
    = static_cast<int16_t>(scale * setAngleToStroke(  rad2Deg * _angles[8], leg.table));  //ankle

}

void seed_converter::TypeG2Arm::Stroke2Angle
(std::vector<double>& _angles, const std::vector<int16_t>& _strokes)
{
  static const float deg2Rad = M_PI / 180.0;
  static const float scale_inv = 0.01;

  _angles[0] = deg2Rad * scale_inv * _strokes[0];
  _angles[1] = deg2Rad * setStrokeToAngle(scale_inv * _strokes[1], shoulder_p.inv_table);
  _angles[2] = (M_PI/2) + deg2Rad * setStrokeToAngle(scale_inv * _strokes[2], elbow_p.inv_table);
  _angles[3] = deg2Rad * scale_inv * _strokes[3];
  _angles[4]
    = deg2Rad * setStrokeToAngle(scale_inv*(_strokes[5] + _strokes[4])*0.5, wrist_p.inv_table);
  _angles[5]
    = deg2Rad * setStrokeToAngle(scale_inv*(_strokes[5] - _strokes[4])*0.5, wrist_r.inv_table);
  _angles[6] = -deg2Rad * (scale_inv * _strokes[6] * 5.556 - 50.0);

  _angles[7] = -deg2Rad * setStrokeToAngle(scale_inv * _strokes[7], leg.inv_table);  // knee
  _angles[8] = deg2Rad * setStrokeToAngle(scale_inv * _strokes[8], leg.inv_table);  // ankle

}

PLUGINLIB_EXPORT_CLASS(seed_converter::TypeG2Arm, seed_converter::StrokeConverter)
