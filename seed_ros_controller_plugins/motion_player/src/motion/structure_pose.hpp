#pragma once

#include "common.hpp"
#include "structure_robotstate.hpp"

class Pose {
    public:

        Pose(){}

        Pose(std::vector<std::string> jnames):state(jnames){

        }

        void reset(std::vector<std::string> jnames){
            state.set_jnames(jnames);
        }

        std::string name;
        RobotState state;//!< 状態
};
