#include <boost/bind.hpp>
#include "odometry.hpp"

Odometry::Odometry(size_t rolling_window_size) :
        last_update_timestamp_(0.0) {
    odom.x = 0;
    odom.y = 0;
    odom.th = 0;
    x.init(rolling_window_size);
    y.init(rolling_window_size);
    th.init(rolling_window_size);

}

void Odometry::init(const ros::Time &time) {
    // Reset accumulators and timestamp:
    last_update_timestamp_ = time;
    odom.x = 0;
    odom.y = 0;
    odom.th = 0;
}

bool Odometry::update(const double &vx, const double &vy, const double &vth, const ros::Time &time) {
    /// Compute x, y and heading using velocity
    const double dt = (time - last_update_timestamp_).toSec();
    if (dt < 0.0001)
        return false; // Interval too small to integrate with

    last_update_timestamp_ = time;

    updateOdometry(dt, vx, vy, vth);

    x.update(dt,vx);
    y.update(dt,vy);
    th.update(dt,vth);
    return true;
}

void Odometry::updateOdometry(double dt, double vx, double vy, double vth) {
    //ロボット座標から見た速度を、ワールド座標から見た速度に変換
    double delta_x = (vx * cos(odom.th) - vy * sin(odom.th)) * dt;
    double delta_y = (vx * sin(odom.th) + vy * cos(odom.th)) * dt;
    double delta_th = vth * dt;

    odom.x += delta_x;
    odom.y += delta_y;
    odom.th += delta_th;
}
