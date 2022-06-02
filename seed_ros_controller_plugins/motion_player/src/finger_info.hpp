#pragma once

#include <vector>
#include "archive.hpp"
#include "fileio.hpp"


struct FingerInfo {
    int msid;
    int sendid;
    int min;
    int max;

protected:
    friend class access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(msid);
        ar & ARCHIVE_NAMEDVALUE(sendid);
        ar & ARCHIVE_NAMEDVALUE(min);
        ar & ARCHIVE_NAMEDVALUE(max);

    }
};

struct FingerInfoList {
    std::vector<FingerInfo> info;

protected:
    friend class access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(info);
        if (ar.getDirection() == T::DIRECTION_INPUT) {
            LOG_INFO_STREAM("FingerInfo was Loaded" << LOG_END);
            for (auto _info:info) {
                LOG_INFO_STREAM("     ms:" << _info.msid << " sendid: " << _info.sendid << " min: " << _info.min << " max: " << _info.max << LOG_END);
            }
        }
    }
};
