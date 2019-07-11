/// @author Sasabuchi Kazuhiro, Shintaro Hori, Hiroaki Yaguchi

#ifndef NOID_NAVIGATION_NOID_MOVE_BASE_H_
#define NOID_NAVIGATION_NOID_MOVE_BASE_H_

#include <vector>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "noid_lower_controller.h"
#include <noid_robot_hardware.h>
namespace mover_robot_hardware
{

/// @brief wheel velocities and goal time
struct wheels
{
  std::vector<float> velocities;

  float time;
};

/// @brief 2D pose
struct pose
{
  float x;

  float y;

  float theta;
};

struct goal
{
  std::vector<float> max_vel;

  std::vector<float> wheel_dV;

  float run_time;

  float warm_up_time;
};

struct states
{
  std::vector<double> cur_vel;

  float cur_time;

  pose moved_distance;

  bool wheel_on;
};

/// @brief Base class of base movement
///
/// This class provides prototype of move base functions for
/// vehicle-type base.
/// Implementations of each hardwares must locate under
/// aero_description/{hardware_type}.
/// aero_description/aero_wheels/controllers/AeroBaseControllers.cc
/// is sample of implementation.
class MoverRobotHW
{
 public: explicit MoverRobotHW(const ros::NodeHandle& _nh, noid_robot_hardware::NoidRobotHW *_in_hw);

 public: ~MoverRobotHW();

 public: void cmdVelCallback(const geometry_msgs::TwistConstPtr& _cmd_vel);

 public: void safetyCheckCallback(const ros::TimerEvent& _event);

 public: void calculateOdometry(const ros::TimerEvent& _event);

 public: void velocityToWheel(double _linear_x, double _linear_y, double _angular_z, std::vector<int16_t>& _wheel_vel);


  /// @param node handle
 private: ros::NodeHandle nh_;

  /// @param rate for move base action
 private: double ros_rate_;

  /// @param subscriber for `cmd_vel`
 private: ros::Subscriber cmd_vel_sub_;
 private: ros::CallbackQueue base_queue_;
 private: ros::AsyncSpinner base_spinner_;
 private: ros::SubscribeOptions base_ops_;

  /// @param current (x, y, theta) (vx, vy, vtheta)
 private: double vx_, vy_, vth_, x_, y_, th_;

  /// @param storing current and last time when msg recieved
 private: ros::Time current_time_, last_time_;

  /// @param tf broadcaster for odom
 private: tf::TransformBroadcaster odom_broadcaster_;

  /// @param odom publisher
 private: ros::Publisher odom_pub_;

  /// @param timer for odom
 private: ros::Timer odom_timer_;

  /// @param rate for odom
 private: double odom_rate_;

  /// @param servo status
 private: bool servo_;

  /// @param timer for safety check
 private: ros::Timer safe_timer_;

  /// @param rate for safety check
 private: double safe_rate_;

  /// @param max duration for safety stop
 private: double safe_duration_;

  /// @param time stamp of the latest recieved cmd_vel msg
 private: ros::Time time_stamp_;

 private: geometry_msgs::Twist prev_cmd_;

 private: boost::mutex base_mtx_;
  ///
 private: noid_robot_hardware::NoidRobotHW *hw_;


};

typedef std::shared_ptr<MoverRobotHW> MoverRobotHWPtr;

}  // mover

}  //navigation


#endif
