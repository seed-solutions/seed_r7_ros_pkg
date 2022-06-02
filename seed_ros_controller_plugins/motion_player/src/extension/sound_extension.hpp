#pragma once

#include "common.hpp"
#include <func_extension.hpp>
#include <QtWidgets>
#include <QAudioDeviceInfo>
#include "audio_player.hpp"
#include "audio_buffer.hpp"

class SoundExtension: public Extension {
public:
    SoundExtension(QObject *parent = Q_NULLPTR) {
        auto format = AudioBuffer::format();

        bytebuf = new QByteArray();
        audioPlayer = new AudioPlayer(format);

        makePitchFreqMap();
    }

    void makePitchFreqMap() {
        nameToFreqMap["C2"] = 65.4;
        nameToFreqMap["C#2"] = 69.3; //lowここから
        nameToFreqMap["D2"] = 73.4;
        nameToFreqMap["D#2"] = 77.8;
        nameToFreqMap["E2"] = 82.4;
        nameToFreqMap["F2"] = 87.3;
        nameToFreqMap["F#2"] = 92.5;
        nameToFreqMap["G2"] = 98.0;
        nameToFreqMap["G#2"] = 103.8;
        nameToFreqMap["A2"] = 110.0;
        nameToFreqMap["A#2"] = 116.5;
        nameToFreqMap["B2"] = 123.5;
        nameToFreqMap["C3"] = 130.8;
        nameToFreqMap["C#3"] = 138.6;
        nameToFreqMap["D3"] = 146.8;
        nameToFreqMap["D#3"] = 155.6;
        nameToFreqMap["E3"] = 164.8;
        nameToFreqMap["F3"] = 174.6;
        nameToFreqMap["F#3"] = 185.0;
        nameToFreqMap["G3"] = 196.0;
        nameToFreqMap["G#3"] = 207.7;
        nameToFreqMap["A3"] = 220.0;
        nameToFreqMap["A#3"] = 233.1;
        nameToFreqMap["B3"] = 246.9; //midここから
        nameToFreqMap["C4"] = 261.6;
        nameToFreqMap["C#4"] = 277.2;
        nameToFreqMap["D4"] = 293.7;
        nameToFreqMap["D#4"] = 311.1;
        nameToFreqMap["E4"] = 329.6; //lowここまで
        nameToFreqMap["F4"] = 349.2;
        nameToFreqMap["F#4"] = 370.0;
        nameToFreqMap["G4"] = 392.0;
        nameToFreqMap["G#4"] = 415.3;
        nameToFreqMap["A4"] = 440.0;
        nameToFreqMap["A#4"] = 466.2;
        nameToFreqMap["B4"] = 493.9;
        nameToFreqMap["C5"] = 523.3;
        nameToFreqMap["C#5"] = 554.4;
        nameToFreqMap["D5"] = 587.3;
        nameToFreqMap["D#5"] = 622.3;
        nameToFreqMap["E5"] = 659.3;
        nameToFreqMap["F5"] = 698.5;
        nameToFreqMap["F#5"] = 740.0;
        nameToFreqMap["G5"] = 784.0;
        nameToFreqMap["G#5"] = 830.6;
        nameToFreqMap["A5"] = 880.0; //midここまで
        nameToFreqMap["A#5"] = 932.3;
        nameToFreqMap["B5"] = 987.8;
    }

    ~SoundExtension() {
        LOG_INFO_STREAM("closed."<<LOG_END);
        if(th.joinable()){
            th.join();
        }

        delete bytebuf;
        delete audioPlayer;
    }

    std::string get_name() {
        return "sound_extension";
    }

    bool execute(const args_type &vars) {

        std::string var1 = ""; //音階名
        double var2 = 0; //時間[s]
        if (!get_value(vars, &var1, &var2))
            return false;

        double freq = pitchNameToFreq(var1);
        double sec = var2;

        playSound(freq, sec);

        return true;
    }

    double pitchNameToFreq(std::string name) {
        double freq = 440;

        if (nameToFreqMap.find(name) != nameToFreqMap.end()) {
            freq = nameToFreqMap[name];
        }

        return freq;
    }

    void playSound(double freq, double seconds) {

        if(th.joinable()){
            th.join();
        }

        const double VOLUME_MAX = 5;
        const int SAMPLE_RATE = AudioBuffer::format().sampleRate(); //[Hz]
        const double FREQ_CONST = ((2.0 * M_PI) / SAMPLE_RATE);

        bytebuf->resize(seconds * SAMPLE_RATE);

        //データを作成
        for (int i = 0; i < (seconds * SAMPLE_RATE); i++) {
            qreal rad = freq * FREQ_CONST * i;
            qreal volume = qSin(rad) * VOLUME_MAX;
            (*bytebuf)[i] = (quint8) volume;
        }

        th = std::thread([&]() {
            audioPlayer->play(*bytebuf);
        });

    }

private:
    std::thread th;
    AudioPlayer* audioPlayer;
    QByteArray *bytebuf;
    std::map<std::string, double> nameToFreqMap;

};
