#pragma once

#include <QPixmap>

#include "common.hpp"
#include "structure_motion.hpp"
#include "archive.hpp"
#include "audio_buffer.hpp"

#include "fileio_json.hpp"
#include "joint_state_list.hpp"

struct KeyFrameInfo{
    int id;            //<! UIと対応付けるためのID
    int step;          //<! 配置ステップ
    QPixmap snapshot;  //<! スナップショット
    std::string name;  //<! コメントとして利用する名前

protected:
    friend class access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(id);
        ar & ARCHIVE_NAMEDVALUE(step);
        ar & ARCHIVE_NAMEDVALUE(snapshot);
        ar & ARCHIVE_NAMEDVALUE(name);
    }
};

struct AudioInfo{
    int id;               //<! UIと対応付けるためのID
    double sec;           //<! 再生開始時刻

    AudioBuffer buffer;
    double audio_sec_st;  //<! 再生開始箇所
    double audio_sec_end; //<! 再生終了箇所
    double audio_speed;   //<! 再生速度

protected:
    friend class access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(id);
        ar & ARCHIVE_NAMEDVALUE(sec);
        ar & ARCHIVE_NAMEDVALUE(buffer);
        ar & ARCHIVE_NAMEDVALUE(audio_sec_st);
        ar & ARCHIVE_NAMEDVALUE(audio_sec_end);
        ar & ARCHIVE_NAMEDVALUE(audio_speed);
    }
};


struct KeyframeMotion : public MotionBase{
    double dt = 0.0;//!< ステップ時間[s]
    std::list<KeyFrameInfo> keyframe;
    std::list<AudioInfo> audio;
    std::string rfpath;

    JointStateList points;//!< 軌道点列(微調整可能)
    KeyframeMotion(){
        points.clear();
        keyframe.clear();
        audio.clear();
    }

    MotionIterator::Ptr getIterator() const;

    /** brief モーションファイルの内容をすべて読み込む
     *
     *
     */
    void load(std::string file_path){
        std::ifstream rfile;
        rfile.open(file_path);
        if(!rfile){
            LOG_INFO_STREAM("cannot open file : "<<file_path<<LOG_END);
            return;
        }
        std::string str;
        std::string line;
        while(getline(rfile, line)){
            if(line == "---points---"){
                break;
            }
            str+=line;
        }

        // 点列以外のデータを読み込む
        JsonInputArchive ar(str);
        ar >> *this;

        this->rfpath = file_path;
    }

    void loadPoints(bool async = true){
        if(rfpath.empty()){
            return;
        }
        std::ifstream rfile;
        rfile.open(rfpath);
        std::string line;
        while(getline(rfile, line)){
            if(line == "---points---"){
                break;
            }
        }
        //点列データを読み込む
        if (rfile) {
            points.loadStates(rfile,async);
        }

        rfpath.clear();
    }


    void save(std::string file_path) const{
        std::ofstream wfile;
        wfile.open(file_path, std::ios::out);
        if(!wfile){
            LOG_INFO_STREAM("cannot open file : "<<file_path<<LOG_END);
            return;
        }

        // 点列以外のデータを書き込む
        JsonOutputArchive ar(false);
        ar << *this;
        wfile << ar.commit();

        wfile<<std::endl<<"---points---"<<std::endl;

        // 点列データを書き込む
        points.saveStates(wfile);
    }

private:
    friend class access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(dt);
        ar & ARCHIVE_NAMEDVALUE(points);
        ar & ARCHIVE_NAMEDVALUE(keyframe);
        ar & ARCHIVE_NAMEDVALUE(audio);
    }
};


class KeyframeMotionIterator: public MotionIterator {

public:
    KeyframeMotionIterator(const KeyframeMotion &motion);

    ~KeyframeMotionIterator(){
    }

    double getStepTime() override;

    int getMaxStep() override;

    bool hasElement(int step) override;

    bool valueAvailable() override;

    TargetData getValue() override;

private:
    int step = 0; //!< 現在のステップ
    const KeyframeMotion &motion;//!< 元のモーション

};

