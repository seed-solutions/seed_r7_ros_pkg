#pragma once

#include "common.hpp"
#include <QPixmap>
#include <QImage>

std::string pixmapToBase64(const QPixmap& pix);

QPixmap pixmapFromBase64(const std::string& base64);

std::string QImageToBase64(const QImage& img);

QImage QImageFromBase64(const std::string& base64);

