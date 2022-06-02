#include "fileio_qt_tools.hpp"
#include <QBuffer>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string.hpp>

std::string pixmapToBase64(const QPixmap& pix){
    QBuffer buffer;
    buffer.open(QIODevice::WriteOnly);
    pix.save(&buffer, "PNG");
    auto const encoded = buffer.data().toBase64();
    QString base64Image(encoded);
    return base64Image.toStdString();
}

QPixmap pixmapFromBase64(const std::string& base64){
    QByteArray utf8_str = QString::fromStdString(base64).toUtf8();
    QPixmap pixmap;
    pixmap.loadFromData(QByteArray::fromBase64(utf8_str));
    return pixmap;
}

std::string QImageToBase64(const QImage& img){
    QBuffer buffer;
    buffer.open(QIODevice::WriteOnly);
    img.save(&buffer, "JPEG");
    auto const encoded = buffer.data().toBase64();
    QString base64Image(encoded);
    return base64Image.toStdString();
}

QImage QImageFromBase64(const std::string& base64){
    QByteArray utf8_str = QString::fromStdString(base64).toUtf8();
    QImage image;
    image.loadFromData(QByteArray::fromBase64(utf8_str));
    return image;
}
