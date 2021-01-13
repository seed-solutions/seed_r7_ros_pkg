#ifndef _TYPE_C_ARM_STROKE_CONVERTER_H_
#define _TYPE_C_ARM_STROKE_CONVERTER_H_

#include <seed_r7_ros_controller/stroke_converter_base.h>

namespace seed_converter
{

class TypeGArm : public StrokeConverter
{
public:
  void makeTables();
  void Angle2Stroke(std::vector<int16_t>& _strokes, const std::vector<double>& _angles);
  void Stroke2Angle(std::vector<double>& _angles, const std::vector<int16_t>& _strokes);

private:
  ConvertTable shoulder_p,elbow_p,wrist_p,wrist_r,leg;
  const std::string upper_csv_dir = "/typeG_arm";
  const std::string lifter_csv_dir = "/typeG_lifter";
};

}

#endif
