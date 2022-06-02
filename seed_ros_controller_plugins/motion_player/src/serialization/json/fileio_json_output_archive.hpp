#pragma once

#include "common.hpp"
#include "fileio_json_format.hpp"

class JsonOutputArchive: public access{
public:

    JsonOutputArchive(bool array=true):array(array){
    }

    ~JsonOutputArchive(){

    }

    template<class T>
    void operator<<(const T& target){
        JsonFormat json;
        access::save(target,json);

        data_list.push_back(picojson::value(json.obj));
    }

    template<class T>
    void operator<<(const std::shared_ptr<T>& target){
        JsonFormat json;
        access::save(*target,json);

        data_list.push_back(picojson::value(json.obj));
    }

    std::string commit(){
        picojson::value val;
        if(data_list.empty()){
            return "";
        }else if (array) {
            val = picojson::value(data_list);
        }else{
            val = picojson::value(data_list[0]);
        }
        return val.serialize();
    }

    picojson::value get_json_value(){
        if(data_list.empty()){
            return picojson::value();
        }else if (array) {
            return picojson::value(data_list);
        }else{
            return picojson::value(data_list[0]);
        }
    }

    void clear(){
        data_list.clear();
    }

private:
    picojson::array data_list;
    bool array=true;
};
