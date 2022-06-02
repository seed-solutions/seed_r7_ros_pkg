#include "audio_player.hpp"
#include <QEventLoop>

AudioPlayer::AudioPlayer(const QAudioFormat& format)
{

    pa_sample_spec ss;
    ss.format = PA_SAMPLE_U8;
    ss.rate = format.sampleRate();
    ss.channels = format.channelCount();

    int pa_errno;
    std::string app_name = "audio_player";
    std::string stream_name = "play";

    pa = pa_simple_new(NULL, app_name.c_str(), PA_STREAM_PLAYBACK, NULL, stream_name.c_str(), &ss, NULL, NULL, &pa_errno);
    if (!pa) {
        fprintf(stderr, "ERROR: Failed to connect pulseaudio server: %s\n", pa_strerror(pa_errno));
    }

}

AudioPlayer::~AudioPlayer(){

    pa_simple_free(pa);
}


void AudioPlayer::play(const QByteArray &data){


    int pa_errno, pa_result;

    pa_result = pa_simple_write(pa, data.data(), (size_t) data.size(), &pa_errno);
    if (pa_result < 0) {
        fprintf(stderr, "ERROR: Failed to write data to pulseaudio: %s\n", pa_strerror(pa_errno));
        return;
    }
}

void AudioPlayer::play(const std::string &data_base64){
    QByteArray utf8_str = QString::fromStdString(data_base64).toUtf8();
    QByteArray data = QByteArray::fromBase64(utf8_str);
    play(data);
}
