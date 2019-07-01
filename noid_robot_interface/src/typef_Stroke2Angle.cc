/*
 * This file auto-generated from script. Do not Edit!
 * Original : aero_startup/.templates/aero_hardware_interface/Stroke2Angle.hh
 * Original : aero_description/{my_robot}/headers/Stroke2Angle.hh
*/

#include <vector>
#include <algorithm>
#include <stdint.h>
#include <math.h> // for M_PI
#include "typef_Stroke2Angle.hh"

namespace noid
{
  namespace typef
  {

 
    //////////////////////////////////////////////////

    float ShoulderPitchInvTable (float _stroke)
    {
      int roundedStroke = static_cast<int>(_stroke);
      int roundedStrokeIndex = roundedStroke - ArrayShoulderPitchInvTableOffset;
      if(static_cast<int>(ShoulderPitchInvTableCandidates.size() - 1) < roundedStrokeIndex) roundedStrokeIndex = static_cast<int>(ShoulderPitchInvTableCandidates.size() - 1); 
      if(roundedStrokeIndex < 0) roundedStrokeIndex = 0;
      auto ref = ShoulderPitchInvTableCandidates.at(roundedStrokeIndex);
      std::vector<S2AData> candidates = ref.first;
      std::vector<S2AData> appendix = ref.second;

      if (_stroke < 0) {
        if (candidates.size() >= 2)
          if (candidates[0].stroke < candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke >= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke < appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
      }

      else {
        if (candidates.size() >= 2)
          if (candidates[0].stroke > candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke <= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke > appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
 
    //////////////////////////////////////////////////
      }
    }
    float ShoulderRollInvTable (float _stroke)
    {
      int roundedStroke = static_cast<int>(_stroke);
      int roundedStrokeIndex = roundedStroke - ArrayShoulderRollInvTableOffset;
      if(static_cast<int>(ShoulderRollInvTableCandidates.size() - 1) < roundedStrokeIndex) roundedStrokeIndex = static_cast<int>(ShoulderRollInvTableCandidates.size() - 1); 
      if(roundedStrokeIndex < 0) roundedStrokeIndex = 0;
      auto ref = ShoulderRollInvTableCandidates.at(roundedStrokeIndex);
      std::vector<S2AData> candidates = ref.first;
      std::vector<S2AData> appendix = ref.second;

      if (_stroke < 0) {
        if (candidates.size() >= 2)
          if (candidates[0].stroke < candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke >= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke < appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
      }

      else {
        if (candidates.size() >= 2)
          if (candidates[0].stroke > candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke <= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke > appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
 
    //////////////////////////////////////////////////
      }
    }
    float ElbowPitchInvTable (float _stroke)
    {
      int roundedStroke = static_cast<int>(_stroke);
      int roundedStrokeIndex = roundedStroke - ArrayElbowPitchInvTableOffset;
      if(static_cast<int>(ElbowPitchInvTableCandidates.size() - 1) < roundedStrokeIndex) roundedStrokeIndex = static_cast<int>(ElbowPitchInvTableCandidates.size() - 1); 
      if(roundedStrokeIndex < 0) roundedStrokeIndex = 0;
      auto ref = ElbowPitchInvTableCandidates.at(roundedStrokeIndex);
      std::vector<S2AData> candidates = ref.first;
      std::vector<S2AData> appendix = ref.second;

      if (_stroke < 0) {
        if (candidates.size() >= 2)
          if (candidates[0].stroke < candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke >= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke < appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
      }

      else {
        if (candidates.size() >= 2)
          if (candidates[0].stroke > candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke <= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke > appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
 
    //////////////////////////////////////////////////
      }
    }
    float WristPitchInvTable (float _stroke)
    {
      int roundedStroke = static_cast<int>(_stroke);
      int roundedStrokeIndex = roundedStroke - ArrayWristPitchInvTableOffset;
      if(static_cast<int>(WristPitchInvTableCandidates.size() - 1) < roundedStrokeIndex) roundedStrokeIndex = static_cast<int>(WristPitchInvTableCandidates.size() - 1); 
      if(roundedStrokeIndex < 0) roundedStrokeIndex = 0;
      auto ref = WristPitchInvTableCandidates.at(roundedStrokeIndex);
      std::vector<S2AData> candidates = ref.first;
      std::vector<S2AData> appendix = ref.second;

      if (_stroke < 0) {
        if (candidates.size() >= 2)
          if (candidates[0].stroke < candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke >= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke < appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
      }

      else {
        if (candidates.size() >= 2)
          if (candidates[0].stroke > candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke <= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke > appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
 
    //////////////////////////////////////////////////
      }
    }
    float WristRollInvTable (float _stroke)
    {
      int roundedStroke = static_cast<int>(_stroke);
      int roundedStrokeIndex = roundedStroke - ArrayWristRollInvTableOffset;
      if(static_cast<int>(WristRollInvTableCandidates.size() - 1) < roundedStrokeIndex) roundedStrokeIndex = static_cast<int>(WristRollInvTableCandidates.size() - 1); 
      if(roundedStrokeIndex < 0) roundedStrokeIndex = 0;
      auto ref = WristRollInvTableCandidates.at(roundedStrokeIndex);
      std::vector<S2AData> candidates = ref.first;
      std::vector<S2AData> appendix = ref.second;

      if (_stroke < 0) {
        if (candidates.size() >= 2)
          if (candidates[0].stroke < candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke >= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke < appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
      }

      else {
        if (candidates.size() >= 2)
          if (candidates[0].stroke > candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke <= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke > appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
 
    //////////////////////////////////////////////////
      }
    }
    float WaistPitchInvTable (float _stroke)
    {
      int roundedStroke = static_cast<int>(_stroke);
      int roundedStrokeIndex = roundedStroke - ArrayWaistPitchInvTableOffset;
      if(static_cast<int>(WaistPitchInvTableCandidates.size() - 1) < roundedStrokeIndex) roundedStrokeIndex = static_cast<int>(WaistPitchInvTableCandidates.size() - 1); 
      if(roundedStrokeIndex < 0) roundedStrokeIndex = 0;
      auto ref = WaistPitchInvTableCandidates.at(roundedStrokeIndex);
      std::vector<S2AData> candidates = ref.first;
      std::vector<S2AData> appendix = ref.second;

      if (_stroke < 0) {
        if (candidates.size() >= 2)
          if (candidates[0].stroke < candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke >= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke < appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
      }

      else {
        if (candidates.size() >= 2)
          if (candidates[0].stroke > candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke <= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke > appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
 
    //////////////////////////////////////////////////
      }
    }
    float WaistRollInvTable (float _stroke)
    {
      int roundedStroke = static_cast<int>(_stroke);
      int roundedStrokeIndex = roundedStroke - ArrayWaistRollInvTableOffset;
      if(static_cast<int>(WaistRollInvTableCandidates.size() - 1) < roundedStrokeIndex) roundedStrokeIndex = static_cast<int>(WaistRollInvTableCandidates.size() - 1); 
      if(roundedStrokeIndex < 0) roundedStrokeIndex = 0;
      auto ref = WaistRollInvTableCandidates.at(roundedStrokeIndex);
      std::vector<S2AData> candidates = ref.first;
      std::vector<S2AData> appendix = ref.second;

      if (_stroke < 0) {
        if (candidates.size() >= 2)
          if (candidates[0].stroke < candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke >= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke < appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
      }

      else {
        if (candidates.size() >= 2)
          if (candidates[0].stroke > candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke <= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke > appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
 
    //////////////////////////////////////////////////
      }
    }
    float NeckPitchInvTable (float _stroke)
    {
      int roundedStroke = static_cast<int>(_stroke);
      int roundedStrokeIndex = roundedStroke - ArrayNeckPitchInvTableOffset;
      if(static_cast<int>(NeckPitchInvTableCandidates.size() - 1) < roundedStrokeIndex) roundedStrokeIndex = static_cast<int>(NeckPitchInvTableCandidates.size() - 1); 
      if(roundedStrokeIndex < 0) roundedStrokeIndex = 0;
      auto ref = NeckPitchInvTableCandidates.at(roundedStrokeIndex);
      std::vector<S2AData> candidates = ref.first;
      std::vector<S2AData> appendix = ref.second;

      if (_stroke < 0) {
        if (candidates.size() >= 2)
          if (candidates[0].stroke < candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke >= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke < appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
      }

      else {
        if (candidates.size() >= 2)
          if (candidates[0].stroke > candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke <= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke > appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
 
    //////////////////////////////////////////////////
      }
    }
    float NeckRollInvTable (float _stroke)
    {
      int roundedStroke = static_cast<int>(_stroke);
      int roundedStrokeIndex = roundedStroke - ArrayNeckRollInvTableOffset;
      if(static_cast<int>(NeckRollInvTableCandidates.size() - 1) < roundedStrokeIndex) roundedStrokeIndex = static_cast<int>(NeckRollInvTableCandidates.size() - 1); 
      if(roundedStrokeIndex < 0) roundedStrokeIndex = 0;
      auto ref = NeckRollInvTableCandidates.at(roundedStrokeIndex);
      std::vector<S2AData> candidates = ref.first;
      std::vector<S2AData> appendix = ref.second;

      if (_stroke < 0) {
        if (candidates.size() >= 2)
          if (candidates[0].stroke < candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke >= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke < appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
      }

      else {
        if (candidates.size() >= 2)
          if (candidates[0].stroke > candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke <= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke > appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
 
    //////////////////////////////////////////////////
      }
    }
    float LegInvTable (float _stroke)
    {
      int roundedStroke = static_cast<int>(_stroke);
      int roundedStrokeIndex = roundedStroke - ArrayLegInvTableOffset;
      if(static_cast<int>(LegInvTableCandidates.size() - 1) < roundedStrokeIndex) roundedStrokeIndex = static_cast<int>(LegInvTableCandidates.size() - 1); 
      if(roundedStrokeIndex < 0) roundedStrokeIndex = 0;
      auto ref = LegInvTableCandidates.at(roundedStrokeIndex);
      std::vector<S2AData> candidates = ref.first;
      std::vector<S2AData> appendix = ref.second;

      if (_stroke < 0) {
        if (candidates.size() >= 2)
          if (candidates[0].stroke < candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke >= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke < appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
      }

      else {
        if (candidates.size() >= 2)
          if (candidates[0].stroke > candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke <= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke > appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
      }
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
}

