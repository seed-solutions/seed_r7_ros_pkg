#include <vector>
#include <algorithm>
#include <stdint.h>
#include <math.h> // for M_PI
#include "stroke_to_angle.h"

namespace noid
{
  namespace common
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
   

  }
}

