#pragma once

#include "common.hpp"

#include <boost/optional.hpp>
#include "structure_robotstate.hpp"

//TODO 指の本数を変えられるようにする
constexpr int OTAMAGRASP_FINGER_NUM = 2;
constexpr int FINGERBOARD_FINGER_NUM = 3;
constexpr int FINGER_NUM = FINGERBOARD_FINGER_NUM + OTAMAGRASP_FINGER_NUM;

struct ArmTarget{
        RobotState tgt_state;
        int step_end;
};

struct FingerTarget{
        double tgt_pos; // 閉じ量の割合 0.0〜1.0
        int step_end;
};

struct ExtensionParam{
        std::string name;
        std::vector<std::string> args;
};

struct TargetData{
        double dt; //ステップ時間[s]
        std::vector<ArmTarget> arm_tgt;
        std::vector<boost::optional<FingerTarget>> finger_tgt;
        std::vector<ExtensionParam> extension;
        TargetData(){
            dt = 0.02;
            finger_tgt.resize(FINGER_NUM,boost::none);
        }
};

class MotionIterator{
    public:
        MotionIterator(){}
        virtual ~MotionIterator(){}

        //! 全ステップ数
        virtual int getMaxStep() = 0;

        //! 一ステップの時間
        virtual double getStepTime() = 0;

        //! そのステップに値があるか
        virtual bool hasElement(int step) = 0;

        //! 値が準備できるか
        virtual bool valueAvailable(){return true;}

        //! 値の取得
        virtual TargetData getValue() = 0;

        typedef std::shared_ptr<MotionIterator> Ptr;
};

class MotionBase{
    public:
        std::string name; //モーション名
        virtual ~MotionBase(){};
        virtual MotionIterator::Ptr getIterator() const = 0;
};
