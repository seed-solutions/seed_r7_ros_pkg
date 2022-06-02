#pragma once

#include "common.hpp"
#include "fileio_json_output_archive.hpp"
#include "type_traits.hpp"
#include <QPixmap>

picojson::value toJson(const QPixmap& value);
picojson::value toJson(const QByteArray& value);
picojson::value toJson(const QImage& value);
