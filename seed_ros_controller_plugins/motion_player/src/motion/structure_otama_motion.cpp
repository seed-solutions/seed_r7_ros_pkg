#include "common.hpp"
#include "structure_otama_motion.hpp"
#include "structure_robotstate.hpp"



struct JNAMES_IF{
        JNAMES_IF(std::vector<std::string> jnames):jnames(jnames){
        }

        std::vector<std::string> jnames;
};

struct JNAMES_ELSE{
        JNAMES_ELSE(std::vector<std::string> jnames):jnames(jnames){
        }

        std::vector<std::string> jnames;
};


void setRobotStateWrapper(JNAMES_ELSE state,const RobotState& in, RobotState& out){

    out = in;
    out.reset();

    //動かすジョイントだけ抽出
    auto in_actual = in.get_position();
    for(auto jname:state.jnames){
        if(in_actual.find(jname) != in_actual.end()){
            in_actual.erase(jname);
        }
    }
    out.set_position(in_actual);
}

void setRobotStateWrapper(JNAMES_IF state,const RobotState&  in, RobotState& out){

    out = in;
    out.reset();

    auto jpos_all = in.get_position();

    //動かすジョイントだけ抽出
    std::map<std::string, double> in_actual;
    for(auto jname:state.jnames){
        if(jpos_all.find(jname) != jpos_all.end()){
            in_actual[jname] = jpos_all.at(jname);
            LOG_DEBUG_STREAM("joint: "<<jname<<" found"<<LOG_END);
        }else{
            LOG_DEBUG_STREAM("joint: "<<jname<<" not found -> skip"<<LOG_END);
        }
    }

    out.set_position(in_actual);

}

MotionIterator::Ptr OtamaMotion::getIterator() const{
    typedef OtamaMotionIterator iterator;
    return MotionIterator::Ptr(new iterator(*this));
}


OtamaMotionIterator::OtamaMotionIterator(const OtamaMotion &motion) :
        motion(motion) {
}

double OtamaMotionIterator::getStepTime() {
    return motion.dt;
}

int OtamaMotionIterator::getMaxStep() {

    int step_max = 0;

    //ハンドだけ見れば良いはずだが、将来的なことを考えて一応アームも見ておく
    if (motion.arm_motion.size() != 0 && step_max < motion.arm_motion.back().step_end) {
        step_max = motion.arm_motion.back().step_end;
    }

    if (motion.body_motion.size() != 0  && step_max < motion.body_motion.back().step) {
        step_max = motion.body_motion.back().step;
    }

    if(motion.hand_motion.size() != 0 && step_max < motion.hand_motion.back().step){
        step_max = motion.hand_motion.back().step;
    }

    for (auto hand_motion_itr = motion.hand_motion.begin(); hand_motion_itr != motion.hand_motion.end(); ++hand_motion_itr) {
        for (unsigned int idx = 0; idx < hand_motion_itr->finger_tgt.size(); ++idx) {
            int step_open_end = hand_motion_itr->finger_tgt[idx].step_open_end;
            if (step_max < step_open_end) step_max = step_open_end;
        }
    }


    for(auto &audio:motion.audio){
        int step = (audio.sec + (audio.audio_sec_end- audio.audio_sec_st))/motion.dt;
        if(step > step_max){
            step_max = step;
        }
    }

    return step_max;
}


bool OtamaMotionIterator::hasElement(int step) {

    if (step < 0 || step > getMaxStep()) {
        return false;
    }

    this->step = step;

    return true;
}

TargetData OtamaMotionIterator::getValue() {
    TargetData target;
    target.dt = motion.dt;

    //アームの処理
    setArmStepMotion(step, target);

    //ボディーの処理
    setBodyStepMotion(step,target);

    //ハンドの処理
    setHandStepMotion(step, target);

    //音声
    setAudioStepMotion(step, target);

    return target;
}


void OtamaMotionIterator::setAudioStepMotion(int step, TargetData &target){
    int resolution = 1;
    double dt = motion.dt;
    for(auto &audio:motion.audio){
        int step_from_st = step - std::round(audio.sec/dt);
        int endstep_from_st = std::round((audio.audio_sec_end - audio.audio_sec_st)/dt);
        if(step_from_st >= 0 && step_from_st <= endstep_from_st && step_from_st%resolution == 0){
            ExtensionParam param;
            param.name = "audio_extension";

            //最大resolutionステップ分の再生区間を算出
            double play_start_sec = step_from_st * dt + audio.audio_sec_st;
            double play_end_sec = play_start_sec + std::min(endstep_from_st - step_from_st,resolution)*dt;

            param.args.push_back(audio.buffer.getByteBufferBase64(play_start_sec,play_end_sec));
            target.extension.push_back(param);
        }
    }
}

void OtamaMotionIterator::setArmStepMotion(int step, TargetData &target) {

    for (auto arm_motion_itr = motion.arm_motion.begin(); arm_motion_itr != motion.arm_motion.end(); ++arm_motion_itr) {
        if (step == (*arm_motion_itr).step_start) {
            ArmTarget armtgt;
            armtgt.step_end = (*arm_motion_itr).step_end;
            setRobotStateWrapper(JNAMES_IF(motion.arm_jnames),(*arm_motion_itr).state_end,armtgt.tgt_state);
            target.arm_tgt.push_back(armtgt);
            break;
        }else if(step < (*arm_motion_itr).step_start){
            //ソートされているはずなので、不要な部分は見ない
            break;
        }
    }
}


void OtamaMotionIterator::setBodyStepMotion(int step, TargetData &target){
    for (auto body_motion_itr = motion.body_motion.begin(); body_motion_itr != motion.body_motion.end(); ++body_motion_itr) {
        if (step == (*body_motion_itr).step_start) {
            ArmTarget bodytgt;
            bodytgt.step_end = (*body_motion_itr).step;
            setRobotStateWrapper(JNAMES_ELSE(motion.arm_jnames),(*body_motion_itr).state_end,bodytgt.tgt_state);

            target.arm_tgt.push_back(bodytgt);
            break;
        }else if(step < (*body_motion_itr).step_start){
            //ソートされているはずなので、不要な部分は見ない
            break;
        }
    }
}

void OtamaMotionIterator::setHandStepMotion(int step, TargetData &target) {

    //指板の手の動作
    for (unsigned idx = 0; idx < FINGERBOARD_FINGER_NUM; ++idx) {
        for (auto hand_motion_itr = motion.hand_motion.begin(); hand_motion_itr != motion.hand_motion.end(); ++hand_motion_itr) {

            if ((*hand_motion_itr).finger_tgt[idx].step_close_start == step) {
                //クローズ開始ステップの場合
                FingerTarget finger_tgt;

                finger_tgt.step_end = (*hand_motion_itr).step;
                finger_tgt.tgt_pos = HAND_CLOSE_POS;
                target.finger_tgt[idx] = finger_tgt;
                break;

            } else if ((*hand_motion_itr).finger_tgt[idx].step_open_start == step) {

#ifdef SOUND_CONNECTABLE_PROV //TODO 次の音とつなげるための仮機能
                auto nextHandMotionItr = std::next(hand_motion_itr);
                if(nextHandMotionItr != motion.hand_motion.end() && nextHandMotionItr->finger_tgt[idx].step_close_start == step){
                    break;
                }
#endif

                //オープン開始ステップの場合
                FingerTarget finger_tgt;

                finger_tgt.step_end = (*hand_motion_itr).finger_tgt[idx].step_open_end;
                finger_tgt.tgt_pos = HAND_OPEN_POS;
                target.finger_tgt[idx] = finger_tgt;
                break;
            }
        }
    }

    for (auto hand_motion_itr = motion.hand_motion.begin(); hand_motion_itr != motion.hand_motion.end(); ++hand_motion_itr) {
        if((*hand_motion_itr).step ==step){

            //オタマをつかむ方の手(ワウハンド)の動作
            for(unsigned idx = 0; idx < OTAMAGRASP_FINGER_NUM;++idx){
                FingerTarget finger_tgt;
                finger_tgt.step_end = step;//TODO とりあえず、現在ステップを目標ステップとしておく。要修正
                finger_tgt.tgt_pos = (*hand_motion_itr).mouth_open_level;
                target.finger_tgt[FINGERBOARD_FINGER_NUM+idx] = finger_tgt;
            }

            if (motion.beep) {
                //音を鳴らすエクステンションの動作
                for (unsigned idx = 0; idx < FINGERBOARD_FINGER_NUM; ++idx) {
                    if ((*hand_motion_itr).finger_tgt[idx].step_close_start != STEP_NONE) {
                        ExtensionParam param;
                        param.name = "sound_extension";
                        param.args.push_back((*hand_motion_itr).finger_tgt[idx].pitch_name);
                        param.args.push_back(std::to_string(((*hand_motion_itr).finger_tgt[idx].step_open_start - step) * motion.dt));
                        target.extension.push_back(param);
                    }
                }
            }

            break;
        }
    }

}

