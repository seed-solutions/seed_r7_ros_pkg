/// @author Sasabuchi Kazuhiro, Shintaro Hori, Hiroaki Yaguchi

#ifndef MOVER_CONTROLLER_H_
#define MOVER_CONTROLLER_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "seed_r7_lower_controller.h"
#include "seed_r7_robot_hardware.h"


#define MAX_ACC_X 1.0
#define MAX_ACC_Y 1.0
#define MAX_ACC_Z 3.0

struct pose
{
  float x;
  float y;
  float theta;
};

class MoverController
{
 public: 
  explicit MoverController(const ros::NodeHandle& _nh, robot_hardware::RobotHW *_in_hw);
  ~MoverController();

 private: 
  void cmdVelCallback(const geometry_msgs::TwistConstPtr& _cmd_vel);
  void safetyCheckCallback(const ros::TimerEvent& _event);
  void calculateOdometry(const ros::TimerEvent& _event);
  void velocityToWheel(double _linear_x, double _linear_y, double _angular_z, std::vector<int16_t>& _wheel_vel);

  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Time current_time_, last_time_, time_stamp_;
  ros::Timer odom_timer_, safe_timer_;
  ros::Subscriber cmd_vel_sub_;
  tf::TransformBroadcaster odom_broadcaster_;

/*
  ros::SubscribeOptions base_ops_;
  ros::CallbackQueue base_queue_;
  ros::AsyncSpinner base_spinner_;
*/

  double vx_, vy_, vth_, x_, y_, th_;
  double ros_rate_, odom_rate_, safety_rate_, safety_duration_;
  float k1_,k2_;
  int num_of_wheels_;
  bool servo_on_;
  std::vector<std::string> wheel_names_;
  std::vector<int> aero_index_;

  boost::mutex base_mtx_;
  robot_hardware::RobotHW *hw_;

};

//typedef std::shared_ptr<NoidMoverController> NoidMoverControllerPtr;


#endif
