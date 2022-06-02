#pragma once

#include "common.hpp"
#include "fileio_json_format.hpp"

class JsonInputArchive: public access{
public:

    JsonInputArchive(const picojson::value& val){
        if (val.is<picojson::object>()) {
            this->obj = val.get<picojson::object>();
            idx = -1;
        } else if (val.is<picojson::array>()) {
            this->array = val.get<picojson::array>();
            idx = 0;
        }
    }

    JsonInputArchive(const std::string &str){

        // JSONデータを解析する。
        picojson::value v;
        const std::string err = picojson::parse(v, str);
        if (err.empty() == false) {
            std::cout<<"failed to parse: "<<err<<" str:"<<str<<std::endl;
            array.clear();
            return;
        }

        if (v.is<picojson::array>()) {
            array = v.get<picojson::array>();
            idx = 0;
        } else if (v.is<picojson::object>()) {
            obj = v.get<picojson::object>();
            idx = -1;
        }
    }

    ~JsonInputArchive(){}

    template<class T>
    void operator>>(T& target){
        JsonFormat json;
        if (idx == -1) {
            json.obj = obj;
        } else if (idx >= 0 && static_cast<int>(array.size()) > idx) {
            json.obj = array[idx].get<picojson::object>();
            ++idx;
        }
        access::load(target, json);
    }

    template<class T>
    void operator>>(std::map<std::string,T>& target){
        JsonFormat json;
        if (idx == -1) {
            json.obj = obj;
        } else if (idx >= 0 && static_cast<int>(array.size()) > idx) {
            json.obj = array[idx].get<picojson::object>();
            ++idx;
        }

        //値のところだけ、ロードを再度実行
        for (auto jsonval : json.obj) {
            T data;
            JsonInputArchive archive(jsonval.second);
            archive>>data;
            target.emplace(std::make_pair(jsonval.first,data));
        }
    }

    template<class T>
    void operator>>(std::shared_ptr<T> &target) {
        if (!target) {
            target = std::make_shared<T>();
        }
        this->operator >>(*target);
    }

    template<class T>
    void operator>>(std::shared_ptr<const T> &target) {
        if (!target) {
            target = std::make_shared<T>();
        }
        auto data = std::const_pointer_cast<T>(target);
        this->operator >>(*data);
    }

    int idx = 0;
    picojson::array array;
    picojson::object obj;
};
