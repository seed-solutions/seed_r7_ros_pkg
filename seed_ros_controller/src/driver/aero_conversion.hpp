#pragma once

#include <ros/ros.h>

class AeroConversion{
public:
    AeroConversion(std::vector<unsigned int> aero_index):aero_index(aero_index){
    }

    template<class T>
    void remapAeroToRos(T *_ros, const std::vector<T> &_aero) {
        size_t sizeAll = aero_index.size();

        for (size_t i = 0; i < sizeAll; ++i) {
            if (0 <= aero_index[i] && aero_index[i] < _aero.size()) {
                _ros[i] = _aero[aero_index[i]];
            }
        }
    }

    template<class T>
    void remapRosToAero(std::vector<T> &_aero, const T*_ros) {
        size_t sizeAll = aero_index.size();

        for (size_t i = 0; i < sizeAll; ++i) {
            if (0 <= aero_index[i] && aero_index[i] < _aero.size()) {
                _aero[aero_index[i]] = _ros[i];
            }
        }
    }

private:
    std::vector<unsigned int> aero_index;//<! 各rosデータに対する、aeroコマンドの送信番号

};
