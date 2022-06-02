#pragma once

#include <QBuffer>
#include <QAudioDecoder>
#include "common.hpp"
#include "archive.hpp"

class AudioBuffer:public QObject
{
    Q_OBJECT

public:
    AudioBuffer(const QAudioFormat& format = AudioBuffer::format());

    AudioBuffer(const AudioBuffer& buf):AudioBuffer(buf.format()){
        name = buf.name;
        buffer = buf.buffer;
        sampleRate = buf.sampleRate;
        duration_sec = buf.duration_sec;
    }

    AudioBuffer& operator=(const AudioBuffer& buf){
        name = buf.name;
        buffer = buf.buffer;
        sampleRate = buf.sampleRate;
        duration_sec = buf.duration_sec;
        return *this;
    }

    static QAudioFormat format(){
        const int SAMPLE_RATE= 44100;
        QAudioFormat format;
        format.setSampleRate(SAMPLE_RATE);
        format.setChannelCount(1);//サラウンドのチャンネル モノラルにしておく
        format.setSampleSize(8);//量子化レート 8bit
        format.setCodec("audio/pcm");
        format.setByteOrder(QAudioFormat::LittleEndian);
        format.setSampleType(QAudioFormat::UnSignedInt);
        return format;
    }

    bool read(const QString &filePath);

    double getTotalTimeSec() const;

    QByteArray getByteBuffer(double sec_st, double sec_end) const;

    std::string getByteBufferBase64(double sec_st, double sec_end) const{
        auto encoded = getByteBuffer(sec_st,sec_end).toBase64().toStdString();
        return encoded;
    }


protected:
    friend class access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(name);
        ar & ARCHIVE_NAMEDVALUE(sampleRate);
        ar & ARCHIVE_NAMEDVALUE(duration_sec);
        ar & ARCHIVE_NAMEDVALUE(buffer);
    }

public:
    std::string name = "";
    QByteArray buffer; //データ本体
    int sampleRate = 0;
    double duration_sec = 0;
private:
    QBuffer m_input;
    QAudioDecoder m_decoder;

private slots:
    void onBufferReady();
};
