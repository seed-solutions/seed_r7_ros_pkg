#include "common.hpp"
#include "structure_keyframe_motion.hpp"

MotionIterator::Ptr KeyframeMotion::getIterator() const {
    typedef KeyframeMotionIterator iterator;
    const_cast<KeyframeMotion*>(this)->loadPoints(); // 点列を非同期で読み込む
    return MotionIterator::Ptr(new iterator(*this));
}

KeyframeMotionIterator::KeyframeMotionIterator(const KeyframeMotion &motion) :
        motion(motion) {
}

double KeyframeMotionIterator::getStepTime() {
    return motion.dt;
}

int KeyframeMotionIterator::getMaxStep() {
    int step_max = 0;
    if(motion.points.size()!=0){
        step_max = static_cast<int>(motion.points.size() - 1);
    }

    for(auto &audio:motion.audio){
        int step = (audio.sec + (audio.audio_sec_end- audio.audio_sec_st))/motion.dt;
        if(step > step_max){
            step_max = step;
        }
    }

    return step_max;
}

bool KeyframeMotionIterator::hasElement(int step) {
    if (step < 0 || step > getMaxStep()) {
        return false;
    }
    this->step = step;
    return true;
}

bool KeyframeMotionIterator::valueAvailable(){
    return (step < motion.points.actual_size() || motion.points.actual_size() == motion.points.size());
}

TargetData KeyframeMotionIterator::getValue() {
    TargetData target;
    target.dt = motion.dt;

    if(step < motion.points.size()){

        ArmTarget armtgt;

#if 0//キーフレームのみを利用する場合
        for(auto itr = motion.keyframe.begin();itr != motion.keyframe.end();++itr){
            if(step == 1){
                armtgt.tgt_state = motion.points.at(itr->step);
                armtgt.step_end = itr->step;
                target.arm_tgt.push_back(armtgt);
                break;
            }else if(itr->step == step){
                ++itr;
                armtgt.tgt_state = motion.points.at(itr->step);
                armtgt.step_end = itr->step;
                target.arm_tgt.push_back(armtgt);
                break;
            }
        }
#else
        armtgt.tgt_state = motion.points[step];
        armtgt.step_end = step + 1;
        target.arm_tgt.push_back(armtgt);
#endif
    }

    int resolution = 1;
    double dt = motion.dt;
    for(auto &audio:motion.audio){
        int step_from_st = step - std::round(audio.sec/dt);//オーディオ再生開始をゼロステップとして、現在のステップ
        int endstep_from_st = std::round((audio.audio_sec_end - audio.audio_sec_st)/dt);//オーディオ再生開始時刻をゼロステップとして、オーディオ終了ステップ
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


//    for(auto audio:motion.audio){
//        if(static_cast<int>(audio.sec/dt) == step){
//            ExtensionParam param;
//            param.name = "audio_extension";
//            param.args.push_back(audio.buffer.getByteBufferBase64(audio.audio_sec_st,audio.audio_sec_end));
//            target.extension.push_back(param);
//        }
//    }



    return target;
}

