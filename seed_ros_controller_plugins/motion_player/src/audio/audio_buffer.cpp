#include "audio_buffer.hpp"
#include <QThread>
#include <QEventLoop>
#include <QFile>
#include <iomanip>
double usec_to_sec(int usec){
    return usec/1000000.;
}

double msec_to_sec(int msec){
    return msec/1000.;
}


AudioBuffer::AudioBuffer(const QAudioFormat& format) :
    m_input(&buffer)
{
    m_decoder.setAudioFormat(format);

    m_input.open(QIODevice::WriteOnly);

    connect(&m_decoder, SIGNAL(bufferReady()), this, SLOT(onBufferReady()));
}

//※コンストラクタと、readの呼び出しは、同じスレッドでなければならない
bool AudioBuffer::read(const QString &filePath)
{
    m_decoder.stop();
    buffer.clear();

    QFile m_file;
    m_file.setFileName(filePath);

    if (!m_file.open(QIODevice::ReadOnly))
    {
        return false;
    }


    //ファイルをデコードする
    m_decoder.setSourceDevice(&m_file);
    m_decoder.start();

    QEventLoop loop;
    connect( &m_decoder, SIGNAL(finished()), &loop, SLOT(quit()) );
    loop.exec();

    sampleRate = m_decoder.audioFormat().sampleRate();
    duration_sec = msec_to_sec(m_decoder.duration());

    return true;
}

QByteArray AudioBuffer::getByteBuffer(double sec_st, double sec_end) const{

    QByteArray ret_data;

    int idx_st = std::round(sec_st*sampleRate);
    int idx_end = std::round(sec_end*sampleRate) - 1;

    if (buffer.size() <= idx_end) {
        idx_end = buffer.size() - 1;
    }
    ret_data.resize(idx_end - idx_st + 1);

    //データを作成
    for (int i=0; i<ret_data.size(); i++) {
        ret_data[i] = (quint8)buffer[i + idx_st];
    }

    return ret_data;
}

double AudioBuffer::getTotalTimeSec() const
{
    return duration_sec;
}

void AudioBuffer::onBufferReady() // SLOT
{
    const QAudioBuffer &buffer = m_decoder.read();

    const int length = buffer.byteCount();
    const char *data = buffer.constData<char>();
    m_input.write(data, length);
}
