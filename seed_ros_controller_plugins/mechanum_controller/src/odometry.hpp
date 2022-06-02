#pragma once

#include <ros/time.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>

namespace bacc = boost::accumulators;

class State {
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;
public:

    State(size_t rolling_window_size = 10)
    :accel_acc_(RollingWindow::window_size = rolling_window_size)
    ,jerk_acc_(RollingWindow::window_size = rolling_window_size){

    }

    void init(size_t acc_window_size) {
        vel = 0;
        acc = 0;
        jerk = 0;
        accel_acc_ = RollingMeanAcc(RollingWindow::window_size = acc_window_size);
        jerk_acc_ = RollingMeanAcc(RollingWindow::window_size = acc_window_size);
    }

    void update(double dt, double cur_vel) {
        vel = cur_vel;
        accel_acc_((vel_prev - vel) / dt);
        vel_prev = vel;

        acc = bacc::rolling_mean(accel_acc_);
        jerk_acc_((acc_prev - acc) / dt);
        acc_prev = acc;

        jerk = bacc::rolling_mean(jerk_acc_);
    }

    double getVel() const{
        return vel;
    }

private:
    double vel = 0;
    double acc = 0;
    double jerk = 0;


    RollingMeanAcc accel_acc_;
    RollingMeanAcc jerk_acc_;
    double vel_prev = 0;
    double acc_prev = 0;
};

class Odometry {
public:

    Odometry(size_t rolling_window_size = 10);

    void init(const ros::Time &time);

    bool update(const double &vx, const double &vy, const double &vth, const ros::Time &time);

    double getX() const
    {
      return odom.x;
    }

    double getY() const
    {
      return odom.y;
    }

    double getTheta() const
    {
        return odom.th;
    }

    //ロボットのベースフレーム基準のX方向速度[m/s]
    double getLinearX() const
    {
      return x.getVel();
    }

    //ロボットのベースフレーム基準のY方向速度[m/s]
    double getLinearY() const
    {
        return y.getVel();
    }

    //ロボットのベースフレーム基準の回展速度[rad/s]
    double getAngular() const
    {
        return th.getVel();
    }



private:

    void updateOdometry(double dt, double vx, double vy, double vth);

    ros::Time last_update_timestamp_;



    struct Odom{
        double x;
        double y;
        double th;
    };

    Odom odom;

    State x;
    State y;
    State th;
};
