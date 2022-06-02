#pragma once

#include <ros/ros.h>

class MechanumKinematics{
public:

    MechanumKinematics(){
    }

    void init(double wheel_radius,double tread,double wheel_base){
        k1_ = -sqrt(2) * (sqrt(pow(tread, 2) + pow(wheel_base, 2)) / 2) * sin(M_PI / 4 + atan2(tread / 2, wheel_base / 2)) / wheel_radius;
        k2_ = 1 / wheel_radius;
    }

    void velocityToWheel(double vx, double vy, double vth, double &front_left, double &front_right, double &rear_left, double &rear_right) {

        // ホイールの位置は、台車の下から見たものなので、注意
        front_left  = (k2_ * (vx - vy) + k1_ * vth);
        front_right = (k2_ * (vx + vy) - k1_ * vth);
        rear_left   = (k2_ * (vx + vy) + k1_ * vth);
        rear_right  = (k2_ * (vx - vy) - k1_ * vth);

        //右の2つは、モータの向きが逆
        front_right *=-1;
        rear_right *=-1;
    }

    void wheelToVelocity(double front_left,double front_right,double rear_left,double rear_right,double &vx, double &vy, double &vth){

        //右の2つは、モータの向きが逆
        front_right *=-1;
        rear_right *=-1;

        vx  =  ( front_left + front_right + rear_left + rear_right) / (4 * k2_);
        vy  =  (-front_left + front_right + rear_left - rear_right) / (4 * k2_);
        vth =  ( front_left - front_right + rear_left - rear_right) / (4 * k1_);
    }

private:
    double k1_ = 0;
    double k2_ = 0;
};
