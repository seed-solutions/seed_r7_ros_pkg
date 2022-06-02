#include "from_json_qt.hpp"
#include "fileio_qt_tools.hpp"

//各型から、picojsonへの変換

void fromJson(picojson::value json, QPixmap &value){
    std::string str = json.get<std::string>();
    value = pixmapFromBase64(str);
}

void fromJson(picojson::value json, QImage &value){
    std::string str = json.get<std::string>();
    value = QImageFromBase64(str);
}

void fromJson(picojson::value json, QByteArray &value){
    value = QByteArray::fromBase64(QString::fromStdString(json.get<std::string>()).toLocal8Bit());
}


