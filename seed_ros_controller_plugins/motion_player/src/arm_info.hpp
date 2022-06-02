#pragma once

#include <vector>
#include <string>

namespace motion_player {

struct JointInfo{
    int sendid;
    std::string name;
};

struct ArmInfo{
    int msid;
    std::vector<JointInfo> jinfo;

    std::vector<std::string> getJointNames() const{
        std::vector<std::string> ret;
        for (auto &data : jinfo) {
            ret.push_back(data.name);
        }
        return ret;
    }
};


struct ArmInfoList{
    std::vector<ArmInfo> info;

    std::vector<std::string> getJointNames() const{
        std::vector<std::string> ret;
        for (auto &data : info) {
            auto jnames = data.getJointNames();
            ret.insert(ret.end(), jnames.begin(), jnames.end());
        }
        return ret;
    }
};
}
