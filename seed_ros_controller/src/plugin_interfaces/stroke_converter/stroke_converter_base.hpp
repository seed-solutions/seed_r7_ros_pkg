#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>

namespace seed_converter {

struct StrokeMap {
    float angle;
    float stroke;
    float range;
};

struct ConvertTable {
    std::vector<StrokeMap> table;
    std::vector<StrokeMap> inv_table;
};

struct DiffJoint {
    float one;
    float two;
};

class StrokeConverter {
public:
    StrokeConverter() {
    }

    virtual ~StrokeConverter() {
    }

    bool initialize(ros::NodeHandle &_nh) {
        if (!_nh.hasParam("csv_config_dir"))
            return false;

        _nh.getParam("csv_config_dir", file_path_);  // e.g. $(find package)/config/csv

        ROS_INFO("start to make stroke convert table");
        makeTables();
        ROS_INFO("finish to make stroke convert table");

        return true;
    }

    virtual void setJointNames(const std::vector<std::string> &names) = 0;
    virtual void makeTables()=0;
    virtual void Angle2Stroke(std::vector<int16_t> &_strokes, const std::vector<double> &_angles)=0;
    virtual void Stroke2Angle(std::vector<double> &_angles, const std::vector<int16_t> &_strokes)=0;
    virtual void calcActuatorVel(std::vector<int16_t> &actuator_vel, const std::vector<double> &joint_vel) = 0;
    virtual void calcJointVel(std::vector<double> &joint_vel,const std::vector<int16_t> &actuator_vel) = 0;

protected:
    bool makeTable(std::vector<seed_converter::StrokeMap> &_table, const std::string _file_name) {
        _table.clear();

        std::ifstream ifs_0(file_path_ + "/" + _file_name, std::ios_base::in);
        if (!ifs_0.is_open()) {
            ROS_ERROR("can't find %s at %s", _file_name.c_str(), file_path_.c_str());
            return false;
        }

        std::string str;
        while (getline(ifs_0, str)) {
            size_t pos = str.find(',');
            if (pos == std::string::npos) {
                ROS_ERROR("%s has invalid structure", _file_name.c_str());
                return false;
            }
            seed_converter::StrokeMap sm;
            sm.angle = std::stof(str.substr(0, pos));
            sm.stroke = std::stof(str.substr(pos + 1));
            _table.push_back(sm);
        }

        //sort in ascending order
        if (_table.at(1).angle < _table.at(0).angle)
            std::reverse(_table.begin(), _table.end());

        _table.front().range = 0;
        for (size_t i = 1; i < _table.size(); ++i) {
            _table.at(i).range = _table.at(i).stroke - _table.at(i - 1).stroke;
        }

        return true;
    }

    void makeInvTable(std::vector<seed_converter::StrokeMap> &_inv_table, const std::vector<seed_converter::StrokeMap> &_table) {
        _inv_table.clear();
        _inv_table = _table;

        //sort in ascending order
        if (_table.at(1).stroke < _table.at(0).stroke)
            std::reverse(_inv_table.begin(), _inv_table.end());

        _inv_table.front().range = 0;
        int sign = (_table.at(1).range < 0 ? -1 : 1);
        for (size_t i = 1; i < _inv_table.size(); ++i) {
            _inv_table.at(i).range = sign * (_inv_table.at(i).stroke - _inv_table.at(i - 1).stroke);
        }
    }

    float setAngleToStroke(const float _angle, const std::vector<seed_converter::StrokeMap> &_table) {
        seed_converter::StrokeMap angle;
        angle.angle = _angle;
        //limit
        if (_angle < _table.front().angle)
            angle.angle = _table.front().angle;
        if (_angle > _table.back().angle)
            angle.angle = _table.back().angle;

        auto ref = std::upper_bound(_table.begin() + 1, _table.end() - 1, angle, [](seed_converter::StrokeMap x, seed_converter::StrokeMap y) -> bool {
            return x.angle < y.angle;
        });

        return ref->stroke - (ref->angle - angle.angle) * ref->range;
    }

    float setStrokeToAngle(const float _stroke, const std::vector<seed_converter::StrokeMap> &_inv_table) {
        seed_converter::StrokeMap stroke;
        stroke.stroke = _stroke;
        //limit
        if (_stroke < _inv_table.front().stroke)
            stroke.stroke = _inv_table.front().stroke;
        if (_stroke > _inv_table.back().stroke)
            stroke.stroke = _inv_table.back().stroke;

        auto ref = std::upper_bound(_inv_table.begin() + 1, _inv_table.end() - 1, stroke, [](seed_converter::StrokeMap x, seed_converter::StrokeMap y) -> bool {
            return x.stroke < y.stroke;
        });

        return ref->angle - (ref->stroke - stroke.stroke) / ref->range;
    }

    DiffJoint setDualAngleToStroke(const float _r_angle, const float _p_angle, const std::vector<seed_converter::StrokeMap> &_r_table, const std::vector<seed_converter::StrokeMap> &_p_table, const bool _is_pitch) {
        float stroke1 = setAngleToStroke(_r_angle, _r_table);
        float stroke2 = setAngleToStroke(_p_angle, _p_table);

        if (_is_pitch)
            return {stroke2 + stroke1, stroke1 - stroke2};
        else
            return {stroke2 + stroke1, stroke2 - stroke1};
    }

    std::string file_path_;
};

}
