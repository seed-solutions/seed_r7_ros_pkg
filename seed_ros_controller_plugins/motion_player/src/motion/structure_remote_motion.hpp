#pragma once

#include "common.hpp"
#include "structure_motion.hpp"
#include "archive.hpp"


class RemoteMotion;
class RemoteMotionIterator: public MotionIterator {

    public:
    RemoteMotionIterator(const RemoteMotion &motion);

        double getStepTime();

        int getMaxStep();

        TargetData getValue();

        bool hasElement(int step);

    private:
        int step = 0;
        const RemoteMotion &motion;
};

struct RemoteMotion : public MotionBase{
        double dt;//!< ステップ時間[s]
        std::map<int,TargetData> motion;

        void reset(){
            motion.clear();
        }

        void fromIterator(MotionIterator::Ptr itr){
            motion.clear();
            int maxStep = itr->getMaxStep();
            for(int step = 0;step<maxStep;++step){
                if(itr->hasElement(step)){
                    TargetData tgt = itr->getValue();
                    motion.insert(std::make_pair(step,tgt));
                    dt = tgt.dt;
                }
            }
        }

        typedef RemoteMotionIterator iterator;
        MotionIterator::Ptr getIterator() const{
            return MotionIterator::Ptr(new iterator(*this));
        }

private:
        friend class access;
        template<class T>
        void serialize(T &ar) {
            ar & ARCHIVE_NAMEDVALUE(dt);
            ar & ARCHIVE_NAMEDVALUE(motion);
        }
};
