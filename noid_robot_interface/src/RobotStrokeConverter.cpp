#include "RobotStrokeConverter.hh"

using namespace noid;
using namespace ROBOT_TYPE;
using namespace common;
StrokeConverter::StrokeConverter()
{
    _define_type = ROBOT_TYPE;
}

StrokeConverter::~StrokeConverter(){}

void StrokeConverter::convert_Angle2Stroke(std::vector<int16_t>& _strokes, const std::vector<double> _angles)
{
    Angle2Stroke(_strokes, _angles);
}

void StrokeConverter::convert_Stroke2Angle(std::vector<double& _angles, const std::vector<int16_t> _strokes)
{ 
    Stroke2Angle(_angles, _strokes);
}

std::string getRobotType()
{
    return _define_type;
}

