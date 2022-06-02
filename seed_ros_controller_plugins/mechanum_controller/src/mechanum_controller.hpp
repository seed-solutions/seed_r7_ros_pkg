#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "mechanum_kinematics.hpp"
#include "odometry.hpp"
#include "speed_limiter.hpp"

namespace mechanum_controller{

  class MechanumController
      : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    MechanumController();

    bool init(hardware_interface::VelocityJointInterface*  hw,
               ros::NodeHandle& root_nh,
               ros::NodeHandle &controller_nh) override;

    void update(const ros::Time& time, const ros::Duration& period) override;

    void starting(const ros::Time& time) override;

    void stopping(const ros::Time& time) override;

  private:

    void updateOdometry(const ros::Time &time);
    void updateCommand(const ros::Time& time, const ros::Duration& period);

    void cmdVelCallback(const geometry_msgs::Twist& command);

    void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  private:
    hardware_interface::JointHandle front_left_wheel_joint;
    hardware_interface::JointHandle front_right_wheel_joint;
    hardware_interface::JointHandle rear_left_wheel_joint;
    hardware_interface::JointHandle rear_right_wheel_joint;

    MechanumKinematics kinematics;
    Odometry odometry;

    std::string base_frame_id_;

    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_; // mapがなくてもodomを確認できるようにする
    bool enable_tf_odom_pub = false;

    //オドメトリのpublish周期関連
    ros::Duration odom_pub_period;
    ros::Time last_odom_pub_time;

    ros::Subscriber sub_cmdvel;
    double cmd_vel_timeout_ = 0.0;// 受信したcmd_velが有効とみなす時間のタイムアウト


    struct CommandTwist
    {
      ros::Time stamp;
      double lin_x;
      double lin_y;
      double ang;

      CommandTwist() :stamp(0.0), lin_x(0.0), lin_y(0.0), ang(0.0) {}
    };


    realtime_tools::RealtimeBuffer<CommandTwist> command_twist_;
    CommandTwist command_struct_twist_;

    /// Speed limiters:
    CommandTwist last1_cmd_;
    CommandTwist last0_cmd_;
    SpeedLimiter limiter_linx_;
    SpeedLimiter limiter_liny_;
    SpeedLimiter limiter_ang_;
  };

}
