#include "robot_stroke_converter.h"

using namespace noid;
using namespace common;

namespace typef
{
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

 //////////////////////////////////////////////////
void Stroke2Angle
    (std::vector<double>& _angles, const std::vector<int16_t> _strokes)
{
      float scale = 0.01;
      float left_wrist_roll_stroke =
        (scale * _strokes[16] + scale * _strokes[17]) * 0.5;
      float right_wrist_roll_stroke =
        (scale * _strokes[8] + scale * _strokes[9]) * 0.5;
      float waist_pitch_stroke =
        (scale * _strokes[19] + scale * _strokes[20]) * 0.5;
      float neck_pitch_stroke =
        (scale * _strokes[1] + scale * _strokes[2]) * 0.5;
      float deg2Rad = M_PI / 180.0;
      float knee_angle  = - deg2Rad * LegInvTable(scale * _strokes[23]);
      float ankle_angle =   deg2Rad * LegInvTable(scale * _strokes[22]);

      // can_order -> ros_order
      _angles[0] =
        deg2Rad * scale * _strokes[21];
      _angles[1] =
        deg2Rad * WaistPitchInvTable(waist_pitch_stroke);
      _angles[2] =
        deg2Rad * WaistRollInvTable(scale * _strokes[19] - waist_pitch_stroke);

      _angles[3] =
        -deg2Rad * ShoulderPitchInvTable(scale * _strokes[11]);
      _angles[4] =
        deg2Rad * ShoulderRollInvTable(scale * _strokes[12]);
      _angles[5] =
        -deg2Rad * scale * _strokes[13];
      _angles[6] =
        -deg2Rad * ElbowPitchInvTable(scale * _strokes[14]);
      _angles[7] =
        -deg2Rad * scale * _strokes[15];
      _angles[8] =
        deg2Rad * WristPitchInvTable(scale * _strokes[16] - left_wrist_roll_stroke);
      _angles[9] =
        -deg2Rad * WristRollInvTable(left_wrist_roll_stroke);
      _angles[10] =
        -deg2Rad * (scale * _strokes[18] * 5.556 - 50.0);
      _angles[11] = 0;
      _angles[12] = 0;
      _angles[13] =
        deg2Rad * (scale * _strokes[18] * 5.556 - 50.0);

      _angles[14] =
        deg2Rad * scale * _strokes[0];
      _angles[15] =
        deg2Rad * NeckPitchInvTable(neck_pitch_stroke);
      _angles[16] =
        -deg2Rad * NeckRollInvTable(scale * _strokes[1] - neck_pitch_stroke);

      _angles[17] =
        -deg2Rad * ShoulderPitchInvTable(scale * _strokes[3]);
      _angles[18] =
        -deg2Rad * ShoulderRollInvTable(scale * _strokes[4]);
      _angles[19] =
        -deg2Rad * scale * _strokes[5];
      _angles[20] =
        -deg2Rad * ElbowPitchInvTable(scale * _strokes[6]);
      _angles[21] =
        -deg2Rad * scale * _strokes[7];
      _angles[22] =
        deg2Rad * WristPitchInvTable(scale * _strokes[8] - right_wrist_roll_stroke);
      _angles[23] =
        deg2Rad * WristRollInvTable(right_wrist_roll_stroke);
      _angles[24] =
        deg2Rad * (scale * _strokes[10] * 5.556 - 50.0);
      _angles[25] = 0;
      _angles[26] = 0;
      _angles[27] =
        -deg2Rad * (scale * _strokes[10] * 5.556 - 50.0);

      _angles[28] = knee_angle;
      _angles[29] = ankle_angle;
}

}






