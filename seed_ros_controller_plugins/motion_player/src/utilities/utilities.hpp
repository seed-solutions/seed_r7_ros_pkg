#pragma once

#include "common.hpp"
#include "unit.hpp"
#include "directory.hpp"

constexpr int USEC_NSSCALE = 1000;
constexpr int MSEC_NSSCALE = 1000*USEC_NSSCALE;
constexpr int SEC_NSSCALE = 1000*MSEC_NSSCALE;

void cyclicsleep(int cycle_ns,struct timespec &time);

void millisleep(int ms);

std::string replaceString(std::string src, std::string tgt, std::string str);

std::vector<std::string> split(const std::string& src, const char* delim);

bool dbl_equal(double a,double b);

bool dbl_equal(double a,double b,double tol);

bool from_stdstring(const std::string &str,std::string* data);
bool from_stdstring(const std::string &str,int* data);
bool from_stdstring(const std::string &str,double* data);

template<class T1,class T2>
std::vector<T2> mapToVec(const std::map<T1,T2>& map){
    std::vector<T2> ret;
    for(auto &m:map){
        ret.push_back(m.second);
    }
    return ret;
}

template<class T1,class T2>
std::vector<T1> mapToKeyVec(const std::map<T1,T2>& map){
    std::vector<T1> ret;
    for(auto &m:map){
        ret.push_back(m.first);
    }
    return ret;
}

std::string getEnvStr(std::string name,std::string defval = "");

double restrictRotation(double deg);

double restrictRotationRad(double rad);

template<class T1,class T2, class T3>
T3 searchKeyValue(T1 key, std::map<T2, T3> item) {

    T3 ret = nullptr;
    auto it = item.find(key);
    if (it != item.end()) {
        ret = it->second;
    }
    return ret;
}

template<class T1, class T2, class T3>
T1 searchValueKey(T3 value, std::map<T1, T2> item) {

    T1 ret = nullptr;
    for (auto it = item.begin(); it != item.end(); ++it) {
        if (it->second == value) {
            ret = it->first;
        }
    }
    return ret;
}

//小数点n桁を切り上げる
double round_n(double number, int n);
