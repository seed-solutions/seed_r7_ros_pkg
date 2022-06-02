#include "from_json.hpp"

//各型から、picojsonへの変換

void fromJson(picojson::value json, int &value){
    value = static_cast<int>(json.get<double>());
}

void fromJson(picojson::value json, bool &value){
    value = json.get<bool>();
}

void fromJson(picojson::value json, double &value){
    value = json.get<double>();
}

void fromJson(picojson::value json, std::string &value){
    value = json.get<std::string>();
}

