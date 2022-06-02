#include "common.hpp"
#include "structure_remote_motion.hpp"

RemoteMotionIterator::RemoteMotionIterator(const RemoteMotion &motion) :
        motion(motion) {
}

double RemoteMotionIterator::getStepTime() {
    return motion.dt;
}

int RemoteMotionIterator::getMaxStep() {
    if(motion.motion.size() ==0)return 0;

    return (motion.motion.rbegin())->first;
}

bool RemoteMotionIterator::hasElement(int step) {
    if (step < 0 || step > getMaxStep()) {
        return false;
    }
    this->step = step;
    return true;
}

TargetData RemoteMotionIterator::getValue() {
    TargetData target;
    target.dt = motion.dt;

    if(motion.motion.count(step) != 0){
        target = motion.motion.at(step);
    }

    return target;
}

