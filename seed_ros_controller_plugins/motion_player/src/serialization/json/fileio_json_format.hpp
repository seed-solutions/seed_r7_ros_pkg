#pragma once

#include "common.hpp"
#include "fileio.hpp"
#include "picojson.h"

struct JsonFormat{
private:
public:
    bool empty(){
        return (obj.size() == 0);
    }

    std::string text(){
        std::string ret = "";
        if (obj.size() != 0) {
            picojson::value jsonval(obj);
            ret = jsonval.serialize();
        }
        return ret;
    }

    picojson::object obj;

};
