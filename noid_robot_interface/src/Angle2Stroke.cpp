/*
 * This file auto-generated from script. Do not Edit!
 * Original : aero_startup/.templates/aero_hardware_interface/Angle2Stroke.hh
 * Original : aero_description/{my_robot}/headers/Angle2Stroke.hh
*/

#include <vector>
#include <algorithm>
#include <stdint.h>
#include <math.h> // for M_PI
#include "Angle2Stroke.h"

namespace noid
{
  namespace common
  {

 
    //////////////////////////////////////////////////

    float ShoulderPitchTable (float _angle)
    {
      int roundedAngle = static_cast<int>(_angle);
      if (_angle > roundedAngle + 0.001) ++roundedAngle;
      int roundedAngleIndex = roundedAngle - ArrayShoulderPitchTableOffset;
      auto ref = ShoulderPitchTableMap.at(roundedAngleIndex);
      float stroke = ref.first;
      float interval = ref.second;

 
    //////////////////////////////////////////////////
      return stroke - (roundedAngle - _angle) * interval;
    }
    float ShoulderRollTable (float _angle)
    {
      int roundedAngle = static_cast<int>(_angle);
      if (_angle > roundedAngle + 0.001) ++roundedAngle;
      int roundedAngleIndex = roundedAngle - ArrayShoulderRollTableOffset;
      auto ref = ShoulderRollTableMap.at(roundedAngleIndex);
      float stroke = ref.first;
      float interval = ref.second;

 
    //////////////////////////////////////////////////
      return stroke - (roundedAngle - _angle) * interval;
    }
    float ElbowPitchTable (float _angle)
    {
      int roundedAngle = static_cast<int>(_angle);
      if (_angle > roundedAngle + 0.001) ++roundedAngle;
      int roundedAngleIndex = roundedAngle - ArrayElbowPitchTableOffset;
      auto ref = ElbowPitchTableMap.at(roundedAngleIndex);
      float stroke = ref.first;
 
    //////////////////////////////////////////////////
      float interval = ref.second;

      return stroke - (roundedAngle - _angle) * interval;
    }
    dualJoint WristRollPitchTable (float _angle1, float _angle2)
    {
      int roundedAngle1 = static_cast<int>(_angle1);
      if (_angle1 > roundedAngle1 + 0.001) ++roundedAngle1;

      int roundedAngle2 = static_cast<int>(_angle2);
      if (_angle2 > roundedAngle2 + 0.001) ++roundedAngle2;

      int roundedAngleIndex1 = roundedAngle1 - ArrayWristRollPitchTableOffset1;
      int roundedAngleIndex2 = roundedAngle2 - ArrayWristRollPitchTableOffset2;
      auto ref1 = WristRollPitchTableMap1.at(roundedAngleIndex1);
      auto ref2 = WristRollPitchTableMap2.at(roundedAngleIndex2);
      float stroke1 = ref1.first, stroke2 = ref2.first;
      float interval1 = ref1.second, interval2 = ref2.second;

      stroke1 -= (roundedAngle1 - _angle1) * interval1;
 
    //////////////////////////////////////////////////
      stroke2 -= (roundedAngle2 - _angle2) * interval2;

      return {stroke2 + stroke1, stroke2 - stroke1} ;
    }
    dualJoint WaistRollPitchTable (float _angle1, float _angle2)
    {
      int roundedAngle1 = static_cast<int>(_angle1);
      if (_angle1 > roundedAngle1 + 0.001) ++roundedAngle1;

      int roundedAngle2 = static_cast<int>(_angle2);
      if (_angle2 > roundedAngle2 + 0.001) ++roundedAngle2;

      int roundedAngleIndex1 = roundedAngle1 - ArrayWaistRollPitchTableOffset1;
      int roundedAngleIndex2 = roundedAngle2 - ArrayWaistRollPitchTableOffset2;
      auto ref1 = WaistRollPitchTableMap1.at(roundedAngleIndex1);
      auto ref2 = WaistRollPitchTableMap2.at(roundedAngleIndex2);
      float stroke1 = ref1.first, stroke2 = ref2.first;
      float interval1 = ref1.second, interval2 = ref2.second;

      stroke1 -= (roundedAngle1 - _angle1) * interval1;
 
    //////////////////////////////////////////////////
      stroke2 -= (roundedAngle2 - _angle2) * interval2;

      return {stroke2 + stroke1, stroke2 - stroke1} ;
    }
    dualJoint NeckRollPitchTable (float _angle1, float _angle2)
    {
      int roundedAngle1 = static_cast<int>(_angle1);
      if (_angle1 > roundedAngle1 + 0.001) ++roundedAngle1;

      int roundedAngle2 = static_cast<int>(_angle2);
      if (_angle2 > roundedAngle2 + 0.001) ++roundedAngle2;

      int roundedAngleIndex1 = roundedAngle1 - ArrayNeckRollPitchTableOffset1;
      int roundedAngleIndex2 = roundedAngle2 - ArrayNeckRollPitchTableOffset2;
      auto ref1 = NeckRollPitchTableMap1.at(roundedAngleIndex1);
      auto ref2 = NeckRollPitchTableMap2.at(roundedAngleIndex2);
      float stroke1 = ref1.first, stroke2 = ref2.first;
      float interval1 = ref1.second, interval2 = ref2.second;

      stroke1 -= (roundedAngle1 - _angle1) * interval1;
      stroke2 -= (roundedAngle2 - _angle2) * interval2;

 
    //////////////////////////////////////////////////
      return {stroke2 + stroke1, stroke2 - stroke1} ;
    }
    float LegTable (float _angle)
    {
      int roundedAngle = static_cast<int>(_angle);
      if (_angle > roundedAngle + 0.001) ++roundedAngle;
      int roundedAngleIndex = roundedAngle - ArrayLegTableOffset;
      auto ref = LegTableMap.at(roundedAngleIndex);
      float stroke = ref.first;
      float interval = ref.second;

      return stroke - (roundedAngle - _angle) * interval;
    }


  }
}

