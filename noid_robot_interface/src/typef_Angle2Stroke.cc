/*
 * This file auto-generated from script. Do not Edit!
 * Original : aero_startup/.templates/aero_hardware_interface/Angle2Stroke.hh
 * Original : aero_description/{my_robot}/headers/Angle2Stroke.hh
*/

#include <vector>
#include <algorithm>
#include <stdint.h>
#include <math.h> // for M_PI
#include "typef_Angle2Stroke.hh"

namespace noid
{
  namespace typef
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
    //////////////////////////////////////////////////
    void Angle2Stroke
    (std::vector<int16_t>& _strokes, const std::vector<double> _angles)
    {
      float rad2Deg = 180.0 / M_PI;
      float scale = 100.0;
      dualJoint right_wrist =
        WristRollPitchTable(rad2Deg * _angles[22],
                            rad2Deg * _angles[23]);
      dualJoint left_wrist =
        WristRollPitchTable(rad2Deg * _angles[8],
                            -rad2Deg * _angles[9]);
      dualJoint waist =
        WaistRollPitchTable(-rad2Deg * _angles[2],
                            rad2Deg * _angles[1]);
      dualJoint neck =
        NeckRollPitchTable(rad2Deg * _angles[16],
                           rad2Deg * _angles[15]);

      // ros_order -> can_order
      _strokes[0] = scale * rad2Deg * _angles[14];
      _strokes[1] = scale * neck.one;
      _strokes[2] = scale * neck.two;

      _strokes[3] =
        scale * ShoulderPitchTable(-rad2Deg * _angles[17]);
      _strokes[4] =
        scale * ShoulderRollTable(-rad2Deg * _angles[18]);
      _strokes[5] = -scale * rad2Deg * _angles[19];
      _strokes[6] = scale * ElbowPitchTable(-rad2Deg * _angles[20]);
      _strokes[7] = -scale * rad2Deg * _angles[21];
      _strokes[8] = scale * right_wrist.one;
      _strokes[9] = scale * right_wrist.two;
      _strokes[10] = -scale * (rad2Deg * _angles[27] - 50.0) * 0.18;

      _strokes[11] =
        scale * ShoulderPitchTable(-rad2Deg * _angles[3]);
      _strokes[12] =
        scale * ShoulderRollTable(rad2Deg * _angles[4]);
      _strokes[13] = -scale * rad2Deg * _angles[5];
      _strokes[14] = scale * ElbowPitchTable(-rad2Deg * _angles[6]);
      _strokes[15] = -scale * rad2Deg * _angles[7];
      _strokes[16] = scale * left_wrist.one;
      _strokes[17] = scale * left_wrist.two;
      _strokes[18] = scale * (rad2Deg * _angles[13] + 50.0) * 0.18;

      _strokes[19] = scale * waist.two;
      _strokes[20] = scale * waist.one;
      _strokes[21] = scale * rad2Deg * _angles[0];

      _strokes[22] = scale * LegTable(  rad2Deg * _angles[29]);
      _strokes[23] = scale * LegTable(- rad2Deg * _angles[28]);
    }

  }
}

