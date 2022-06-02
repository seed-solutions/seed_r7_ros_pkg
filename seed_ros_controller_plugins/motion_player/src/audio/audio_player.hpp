#pragma once

#include "common.hpp"
#include <QBuffer>
#include <QAudioFormat>

#include <pulse/error.h>  /* pulseaudio */
#include <pulse/simple.h> /* pulseaudio */

class AudioPlayer
{
public:
    AudioPlayer(const QAudioFormat& format);
    void play(const QByteArray &data);
    void play(const std::string &data_base64);

    void discard_buffer(){
        int pa_errno, pa_result;
        pa_result = pa_simple_flush(pa, &pa_errno);
        if (pa_result < 0) {
            fprintf(stderr, "ERROR: Failed to flush: %s\n", pa_strerror(pa_errno));
            return;
        }
    }

    void wait_stop(){
        int pa_errno, pa_result;
        pa_result = pa_simple_drain(pa, &pa_errno);
        if (pa_result < 0) {
            fprintf(stderr, "ERROR: Failed to drain: %s\n", pa_strerror(pa_errno));
            return;
        }
    }

    ~AudioPlayer();

private:
    QByteArray m_data;
    pa_simple *pa = nullptr;
};

