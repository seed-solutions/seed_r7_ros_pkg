#include "to_json.hpp"

#include "from_json.hpp"

picojson::value toJson(const int& value) {
    return picojson::value(static_cast<double>(value));
}

picojson::value toJson(const bool &value){
    return picojson::value(value);
}

picojson::value toJson(const double & value){
    return picojson::value(value);
}

picojson::value toJson(const std::string & value){
    if (value.empty()) {
        return picojson::value();
    } else {
        return picojson::value(value);
    }
}
