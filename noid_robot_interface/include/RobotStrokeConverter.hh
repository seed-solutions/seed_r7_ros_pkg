#ifndef _ROBOT_STROKE_CONVERTER_H_
#define _ROBOT_STROKE_CONVERTER_H_

#define ROBOT_TYPE typef

//include file depends on robot_type
#ifndef typef
#include "typef_Angle2Stroke.hh"
#include "typef_Stroke2Angle.hh"
#elif typefcesy
#include "typefcesy_Angle2Stroke.hh"
#include "typefcesy_Stroke2Angle.hh"
#else
#endif

//not depend on robot type
#include "CommonAngle2Stroke.hh"


class StrokeConverter
{
  
public: 
    StrokeConverter(); 
    ~StrokeConverter();
    void convert_Angle2Stroke(std::vector<int16_t>& _strokes, const std::vector<double> _angles);
    void convert_Stroke2Angle(std::vector<double& _angles, const std::vector<int16_t> _strokes);
    std::string getRobotType();
private:
    std::string define_type_;


}

#endif

