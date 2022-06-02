#pragma once

#include "common.hpp"
#include "fileio_json_input_archive.hpp"
#include "type_traits.hpp"
#include <QPixmap>

void fromJson(picojson::value json, QPixmap &value);
void fromJson(picojson::value json, QByteArray &value);
void fromJson(picojson::value json, QImage &value);

