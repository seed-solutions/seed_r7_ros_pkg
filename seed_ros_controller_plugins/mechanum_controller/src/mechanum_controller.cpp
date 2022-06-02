#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>
#include "mechanum_controller.hpp"

namespace mechanum_controller {

MechanumController::MechanumController(): command_struct_twist_(){

}

bool MechanumController::init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    std::string front_left_wheel_name;
    std::string front_right_wheel_name;
    std::string rear_left_wheel_name;
    std::string rear_right_wheel_name;
    if (!controller_nh.getParam("front_left_wheel", front_left_wheel_name) || !controller_nh.getParam("front_right_wheel", front_right_wheel_name) || !controller_nh.getParam("rear_left_wheel", rear_left_wheel_name) || !controller_nh.getParam("rear_right_wheel", rear_right_wheel_name)) {
        ROS_ERROR_STREAM("Couldn't retrieve wheel name");
        return false;
    }

    front_left_wheel_joint = hw->getHandle(front_left_wheel_name);
    front_right_wheel_joint = hw->getHandle(front_right_wheel_name);
    rear_left_wheel_joint = hw->getHandle(rear_left_wheel_name);
    rear_right_wheel_joint = hw->getHandle(rear_right_wheel_name);

    controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);


    double odom_pub_rate;
    controller_nh.param("odom_pub_rate", odom_pub_rate, 50.0);
    controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);

    ROS_INFO_STREAM("odometry will be published at "<< odom_pub_rate << "[Hz]");
    ROS_INFO_STREAM("Velocity commands will be considered old if they are older than "<< cmd_vel_timeout_ << "[s]");

    odom_pub_period = ros::Duration(1.0 / odom_pub_rate);

    setOdomPubFields(root_nh, controller_nh);

    controller_nh.param("linear/x/has_velocity_limits"    , limiter_linx_.has_velocity_limits    , limiter_linx_.has_velocity_limits    );
    controller_nh.param("linear/x/has_acceleration_limits", limiter_linx_.has_acceleration_limits, limiter_linx_.has_acceleration_limits);
    controller_nh.param("linear/x/max_velocity"           , limiter_linx_.max_velocity           ,  limiter_linx_.max_velocity          );
    controller_nh.param("linear/x/min_velocity"           , limiter_linx_.min_velocity           , -limiter_linx_.max_velocity          );
    controller_nh.param("linear/x/max_acceleration"       , limiter_linx_.max_acceleration       ,  limiter_linx_.max_acceleration      );
    controller_nh.param("linear/x/min_acceleration"       , limiter_linx_.min_acceleration       , -limiter_linx_.max_acceleration      );

    controller_nh.param("linear/y/has_velocity_limits"    , limiter_liny_.has_velocity_limits    , limiter_liny_.has_velocity_limits    );
    controller_nh.param("linear/y/has_acceleration_limits", limiter_liny_.has_acceleration_limits, limiter_liny_.has_acceleration_limits);
    controller_nh.param("linear/y/max_velocity"           , limiter_liny_.max_velocity           ,  limiter_liny_.max_velocity          );
    controller_nh.param("linear/y/min_velocity"           , limiter_liny_.min_velocity           , -limiter_liny_.max_velocity          );
    controller_nh.param("linear/y/max_acceleration"       , limiter_liny_.max_acceleration       ,  limiter_liny_.max_acceleration      );
    controller_nh.param("linear/y/min_acceleration"       , limiter_liny_.min_acceleration       , -limiter_liny_.max_acceleration      );

    controller_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
    controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
    controller_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );


    controller_nh.param("enable_tf_odom_pub", enable_tf_odom_pub, enable_tf_odom_pub);

    double wheel_radius = 0;
    double tread = 0;
    double wheel_base = 0;
    controller_nh.getParam("mecha_param/wheel_radius", wheel_radius);
    controller_nh.getParam("mecha_param/tread", tread);
    controller_nh.getParam("mecha_param/wheel_base", wheel_base);

    kinematics.init(wheel_radius, tread, wheel_base);

    sub_cmdvel = controller_nh.subscribe("cmd_vel", 1, &MechanumController::cmdVelCallback, this);
    return true;
}

void MechanumController::update(const ros::Time &time, const ros::Duration &period) {
    updateOdometry(time);
    updateCommand(time, period);
}

void MechanumController::starting(const ros::Time &time) {
}

void MechanumController::stopping(const ros::Time &time) {
}

void MechanumController::updateOdometry(const ros::Time &time) {
    //現在速度を取得
    const double fl_speed = front_left_wheel_joint.getVelocity();
    const double fr_speed = front_right_wheel_joint.getVelocity();
    const double rl_speed = rear_left_wheel_joint.getVelocity();
    const double rr_speed = rear_right_wheel_joint.getVelocity();

    if (std::isnan(fl_speed) || std::isnan(fr_speed) || std::isnan(rl_speed) || std::isnan(rr_speed)) {
        return;
    }

    //オドメトリを更新
    double vx = 0;
    double vy = 0;
    double vth = 0;
    kinematics.wheelToVelocity(fl_speed, fr_speed, rl_speed, rr_speed, vx, vy, vth);
    odometry.update(vx, vy, vth, time);

    //オドメトリのパブリッシュ
    auto next_odom_pub_time = last_odom_pub_time + odom_pub_period;
    if (next_odom_pub_time < time)
    {
        last_odom_pub_time = next_odom_pub_time;

        const geometry_msgs::Quaternion orientation(tf::createQuaternionMsgFromYaw(odometry.getTheta()));

        if (odom_pub_->trylock())
        {
          odom_pub_->msg_.header.stamp = time;
          odom_pub_->msg_.pose.pose.position.x = odometry.getX();
          odom_pub_->msg_.pose.pose.position.y = odometry.getY();
          odom_pub_->msg_.pose.pose.orientation = orientation;
          odom_pub_->msg_.twist.twist.linear.x  = odometry.getLinearX();
          odom_pub_->msg_.twist.twist.linear.y  = odometry.getLinearY();
          odom_pub_->msg_.twist.twist.angular.z = odometry.getAngular();
          odom_pub_->unlockAndPublish();
        }

        if (enable_tf_odom_pub && tf_odom_pub_->trylock())
        {
          geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
          odom_frame.header.stamp = time;
          odom_frame.transform.translation.x = odometry.getX();
          odom_frame.transform.translation.y = odometry.getY();
          odom_frame.transform.rotation = orientation;
          tf_odom_pub_->unlockAndPublish();
        }
    }

}

void MechanumController::updateCommand(const ros::Time &time, const ros::Duration &period) {

    //非RT領域で書き込まれたデータをRT領域に引っ張ってくる
    CommandTwist cmd_twist = *(command_twist_.readFromRT());

    const double dt = (time - cmd_twist.stamp).toSec();

    //現在時刻とかけ離れた時刻にパブリッシュされたデータは無視する
    if (dt > cmd_vel_timeout_)
    {
        cmd_twist.lin_x = 0.0;
        cmd_twist.lin_y = 0.0;
        cmd_twist.ang = 0.0;
    }

    const double cmd_dt(period.toSec());

    //速度、加速度、ジャークのリミットをかける
    limiter_linx_.limit(cmd_twist.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x, cmd_dt);
    limiter_liny_.limit(cmd_twist.lin_y, last0_cmd_.lin_y, last1_cmd_.lin_y, cmd_dt);
    limiter_ang_.limit(cmd_twist.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);
    last1_cmd_ = last0_cmd_;
    last0_cmd_ = cmd_twist;

    double fl_speed;
    double fr_speed;
    double rl_speed;
    double rr_speed;
    kinematics.velocityToWheel(cmd_twist.lin_x, cmd_twist.lin_y, cmd_twist.ang, fl_speed, fr_speed, rl_speed, rr_speed);

    front_left_wheel_joint.setCommand(fl_speed);
    front_right_wheel_joint.setCommand(fr_speed);
    rear_left_wheel_joint.setCommand(rl_speed);
    rear_right_wheel_joint.setCommand(rr_speed);
}

void MechanumController::cmdVelCallback(const geometry_msgs::Twist &command) {
    if (isRunning()) {
        if(std::isnan(command.angular.z) || std::isnan(command.linear.x) || std::isnan(command.linear.y))
        {
          ROS_WARN("Received NaN in geometry_msgs::Twist. Ignoring command.");
          return;
        }


        //コマンドを、リアルタイムとの連携用バッファにつめ直す
        command_struct_twist_.lin_x   = command.linear.x;
        command_struct_twist_.lin_y   = command.linear.y;
        command_struct_twist_.ang   = command.angular.z;
        command_struct_twist_.stamp = ros::Time::now();
        command_twist_.writeFromNonRT (command_struct_twist_);
    }
}

void MechanumController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Get and check params for covariances
  XmlRpc::XmlRpcValue pose_cov_list;
  controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
  ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(pose_cov_list.size() == 6);
  for (int i = 0; i < pose_cov_list.size(); ++i)
    ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  XmlRpc::XmlRpcValue twist_cov_list;
  controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
  ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(twist_cov_list.size() == 6);
  for (int i = 0; i < twist_cov_list.size(); ++i)
    ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

  // Setup odometry realtime publisher + odom message constant fields
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
  odom_pub_->msg_.header.frame_id = "odom";
  odom_pub_->msg_.child_frame_id = base_frame_id_;
  odom_pub_->msg_.pose.pose.position.z = 0;
  odom_pub_->msg_.pose.covariance = {
      static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
      0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
      0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
      0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
      0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
      0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5]) };
  odom_pub_->msg_.twist.twist.linear.y  = 0;
  odom_pub_->msg_.twist.twist.linear.z  = 0;
  odom_pub_->msg_.twist.twist.angular.x = 0;
  odom_pub_->msg_.twist.twist.angular.y = 0;
  odom_pub_->msg_.twist.covariance = {
      static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
      0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
      0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
      0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
      0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
      0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5]) };

  tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
  tf_odom_pub_->msg_.transforms.resize(1);
  tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
  tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
  tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
}

}

PLUGINLIB_EXPORT_CLASS(mechanum_controller::MechanumController, controller_interface::ControllerBase)
