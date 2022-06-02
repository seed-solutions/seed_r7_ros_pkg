#pragma once

#include "common.hpp"
#include "func_extension.hpp"
#include "audio_player.hpp"
#include "audio_buffer.hpp"

class AudioExtension:public Extension
{
public:
    AudioExtension(){

        auto format = AudioBuffer::format();

        QAudioDeviceInfo info(QAudioDeviceInfo::defaultOutputDevice());
        if (!info.isFormatSupported(format)) {
            LOG_ERROR_STREAM("Raw audio format not supported by backend, cannot play audio.");
            return;
        }

        audioPlayer = new AudioPlayer(format);
    }

    ~AudioExtension(){
        delete audioPlayer;
    }

    std::string get_name(){
        return "audio_extension";
    }

    void entry(){
        audioPlayer->discard_buffer();
    }

    void exit(){
        audioPlayer->wait_stop();
    }

    bool execute(const args_type &vars){
        std::string var1 = "";//音声データ
        if(!get_value(vars,&var1))return false;

        audioPlayer->play(var1);

        return true;
    }

private:
    AudioPlayer* audioPlayer;
};
