#ifndef _ROBOT_STROKE_CONVERTER_H_
#define _ROBOT_STROKE_CONVERTER_H_

#include "Angle2Stroke.hh"
#include "Stroke2Angle.hh"
#include "CommonAngle2Stroke.hh"
#include "ros/ros.h"

namespace typef
{
    void Angle2Stroke
    (std::vector<int16_t>& _strokes, const std::vector<double> _angles);
    void Stroke2Angle
    (std::vector<double>& _angles, const std::vector<int16_t> _strokes);
}








