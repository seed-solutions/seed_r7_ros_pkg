#pragma once

#include "common.hpp"

#include "structure_robotstate.hpp"
#include "structure_keyframe_motion.hpp"
#include "archive.hpp"

//音をつなげる機能(仮)
#define SOUND_CONNECTABLE_PROV

const int STEP_NONE = -1;
const int STEP_START = 1;//最小ステップ(ゼロステップは制御に含まれない)
const double MOUTH_CLOSE = 0;
const int ID_NONE = -1;

struct OtamaArmTarget{
        int id;//OtamaHandTargetのidと対応
        int step_start;
        int step_end;
        RobotState state_end;//step_endでのロボットの状態

protected:
    friend class access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(id);
        ar & ARCHIVE_NAMEDVALUE(step_start);
        ar & ARCHIVE_NAMEDVALUE(step_end);
        ar & ARCHIVE_NAMEDVALUE(state_end);
    }
};

struct OtamaBodyTarget:KeyFrameInfo{
        int step_start;
        RobotState state_end;//step_endでのロボットの状態

protected:
    friend class access;
    template<class T>
    void serialize(T &ar) {
        KeyFrameInfo::serialize(ar);
        ar & ARCHIVE_NAMEDVALUE(step_start);
        ar & ARCHIVE_NAMEDVALUE(state_end);
    }

};

struct OtamaFingerTarget {
        int step_close_start;
        int step_open_start;
        int step_open_end;
        std::string pitch_name; // 音名
        OtamaFingerTarget() :
                step_close_start(STEP_NONE), step_open_start(STEP_NONE), step_open_end(STEP_NONE) {
        }

protected:
    friend class access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(step_close_start);
        ar & ARCHIVE_NAMEDVALUE(step_open_start);
        ar & ARCHIVE_NAMEDVALUE(step_open_end);
        ar & ARCHIVE_NAMEDVALUE(pitch_name);
    }
};


struct OtamaHandTarget{


        int id;
        int step;
        std::vector<OtamaFingerTarget> finger_tgt;
        double percent; // 指板上のハンドの位置
        double mouth_open_level;
        OtamaHandTarget(){
            id = 0;
            step = 0;
            percent = 0;
            finger_tgt.resize(FINGER_NUM);
            mouth_open_level = MOUTH_CLOSE;//仮に0を口を閉じている状態としておく
        }

protected:
    friend class access;
    template<class T>
    void serialize(T &ar) {
        ar & ARCHIVE_NAMEDVALUE(id);
        ar & ARCHIVE_NAMEDVALUE(step);
        ar & ARCHIVE_NAMEDVALUE(finger_tgt);
        ar & ARCHIVE_NAMEDVALUE(percent);
        ar & ARCHIVE_NAMEDVALUE(mouth_open_level);
    }
};


enum MusicalAccidentals{
    Sharp,
    Flat,
};

struct OtamaMotion:public MotionBase{
        double dt = 0.02;//!< ステップ時間[s]
        int bpm = 100;

        bool beep = false;

        //拍子記号
        int ts_den = 4; //ベーシックビート
        int ts_num = 4; //拍

        //スケール
        int ms_acc = Sharp;
        int ms_num = 0;


        std::vector<std::pair<std::string,double>> finger1_pitchList; // チューニング結果
        std::vector<double> finger_offset_pos; // 指のオフセット位置

        std::list<OtamaArmTarget> arm_motion;  // オタマに関係するモーション
        std::list<OtamaBodyTarget> body_motion; // オタマに関係しないボディーのモーション
        std::list<OtamaHandTarget> hand_motion;

        std::list<AudioInfo> audio;

        std::vector<std::string> arm_jnames;//オタマを持つアームの動作関節リスト

        std::string grasp_hand = "r_eef_grasp_link";
        std::string finger_hand = "l_eef_grasp_link";

        RobotState default_state;
        QPixmap thumbnail;// default_stateのスナップショット(TODO 削除予定 ロード時にサムネイルを取得)

        double max_dist = 0;

        MotionIterator::Ptr getIterator() const;

        void load(std::string file_path){
            std::ifstream rfile;
            rfile.open(file_path);
            if(!rfile){
                LOG_INFO_STREAM("cannot open file : "<<file_path<<LOG_END);
                return;
            }

            std::string str((std::istreambuf_iterator<char>(rfile)), std::istreambuf_iterator<char>());

            JsonInputArchive ar(str);
            ar >> *this;
        }

        void save(std::string file_path) const{
            std::ofstream wfile;
            wfile.open(file_path, std::ios::out);
            if(!wfile){
                LOG_INFO_STREAM("cannot open file : "<<file_path<<LOG_END);
                return;
            }
            JsonOutputArchive ar(false);
            ar << *this;
            wfile << ar.commit();
        }

private:
        friend class access;
        template<class T>
        void serialize(T &ar) {


            ar & ARCHIVE_NAMEDVALUE(dt);
            ar & ARCHIVE_NAMEDVALUE(bpm);

            ar & ARCHIVE_NAMEDVALUE(ts_den);
            ar & ARCHIVE_NAMEDVALUE(ts_num);

            ar & ARCHIVE_NAMEDVALUE(ms_acc);
            ar & ARCHIVE_NAMEDVALUE(ms_num);

            ar & ARCHIVE_NAMEDVALUE(finger1_pitchList);
            ar & ARCHIVE_NAMEDVALUE(finger_offset_pos);
            ar & ARCHIVE_NAMEDVALUE(arm_motion);
            ar & ARCHIVE_NAMEDVALUE(body_motion);
            ar & ARCHIVE_NAMEDVALUE(hand_motion);
            ar & ARCHIVE_NAMEDVALUE(audio);
            ar & ARCHIVE_NAMEDVALUE(arm_jnames);
            ar & ARCHIVE_NAMEDVALUE(grasp_hand);
            ar & ARCHIVE_NAMEDVALUE(finger_hand);
            ar & ARCHIVE_NAMEDVALUE(default_state);
            ar & ARCHIVE_NAMEDVALUE(thumbnail);
            ar & ARCHIVE_NAMEDVALUE(max_dist);
        }
};


class OtamaMotionIterator: public MotionIterator {

    public:
        OtamaMotionIterator(const OtamaMotion &motion);

        double getStepTime();

        int getMaxStep();

        TargetData getValue();

        bool hasElement(int step);

    private:

        void setArmStepMotion(int step, TargetData &target);
        void setBodyStepMotion(int step, TargetData &target);
        void setHandStepMotion(int step, TargetData &target);
        void setAudioStepMotion(int step, TargetData &target);

    private:
        const double HAND_CLOSE_POS = 1.0;
        const double HAND_OPEN_POS = 0.0;
        int step = 0;
        const OtamaMotion &motion;
};
