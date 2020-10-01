/// @author Sasabuchi Kazuhiro, Shintaro Hori, Hiroaki Yaguchi
#include "seed_r7_ros_controller/seed_r7_mover_controller.h"

robot_hardware::MoverController::MoverController
(const ros::NodeHandle& _nh, robot_hardware::RobotHW *_in_hw) :
  nh_(_nh),hw_(_in_hw),
  vx_(0), vy_(0), vth_(0), x_(0), y_(0), th_(0)//, base_spinner_(1, &base_queue_)
{
  move_base_action_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("/move_base", true);

  //--- calcurate the coefficient(k1_,k2_) for wheel FK--
  float wheel_radius,tread,wheelbase;
  nh_.getParam("/seed_r7_mover_controller/wheel_radius", wheel_radius);
  nh_.getParam("/seed_r7_mover_controller/tread", tread);
  nh_.getParam("/seed_r7_mover_controller/wheelbase", wheelbase);

  k1_ =  - sqrt(2) * ( sqrt( pow(tread,2)+pow(wheelbase,2) )/2 ) * sin( M_PI/4 + atan2(tread/2,wheelbase/2) ) / wheel_radius;
  k2_ = 1 / wheel_radius;
  //---------

  nh_.getParam("/seed_r7_mover_controller/ros_rate", ros_rate_);
  nh_.getParam("/seed_r7_mover_controller/odom_rate", odom_rate_);
  nh_.getParam("/seed_r7_mover_controller/safety_rate", safety_rate_);
  nh_.getParam("/seed_r7_mover_controller/safety_duration", safety_duration_);
  nh_.getParam("/joint_settings/wheel/aero_index", aero_index_);

  num_of_wheels_ = aero_index_.size();
 
  servo_on_ = false;
  current_time_ = ros::Time::now();
  last_time_ = current_time_;

/*
  base_ops_ = ros::SubscribeOptions::create<geometry_msgs::Twist >( "cmd_vel", 2, boost::bind(&NoidMoverController::cmdVelCallback, this, _1),ros::VoidPtr(), &base_queue_);
  cmd_vel_sub_ = nh_.subscribe(base_ops_);
  ros::TimerOptions tmopt(ros::Duration(safety_rate_), boost::bind(&NoidMoverController::safetyCheckCallback,this, _1), &base_queue_);
  safe_timer_ = _nh.createTimer(tmopt);
  base_spinner_.start();
*/

  cmd_vel_sub_ = nh_.subscribe("cmd_vel",1, &MoverController::cmdVelCallback,this);
  safe_timer_ = nh_.createTimer(ros::Duration(safety_rate_), &MoverController::safetyCheckCallback, this);

  // for odometory
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  odom_timer_ = nh_.createTimer(ros::Duration(odom_rate_), &MoverController::calculateOdometry, this);

  initialpose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
  led_control_server_
    = nh_.advertiseService("led_control", &MoverController::ledControlCallback,this);

  set_initialpose_server_
    = nh_.advertiseService("set_initialpose", &MoverController::setInitialPoseCallback,this);

}

//////////////////////////////////////////////////
/// @brief destructor
robot_hardware::MoverController::~MoverController()
{
}

//////////////////////////////////////////////////
/// @brief control with cmd_vel
void robot_hardware::MoverController::cmdVelCallback(const geometry_msgs::TwistConstPtr& _cmd_vel)
{
  ros::Time now = ros::Time::now();
  ROS_DEBUG("cmd_vel: %f %f %f", _cmd_vel->linear.x, _cmd_vel->linear.y, _cmd_vel->angular.z);

  if (base_mtx_.try_lock()) {
    if(hw_->robot_status_.p_stopped_err_)
    {
      //move_base_action_->cancelAllGoals();  //if you want to cancel goal, use this
      vx_ = vy_ = vth_ = 0.0;
    }
    else if (_cmd_vel->linear.x == 0.0 && _cmd_vel->linear.y == 0.0 && _cmd_vel->angular.z == 0.0) {
      vx_ = vy_ = vth_ = 0.0;
    }
    else{
      vx_ = _cmd_vel->linear.x;
      vy_ = _cmd_vel->linear.y;
      vth_ = _cmd_vel->angular.z;
    }
/*
    else {
      double dt = (now - time_stamp_).toSec();
      double acc_x = (_cmd_vel->linear.x  - vx_)  / dt;
      double acc_y = (_cmd_vel->linear.y  - vy_)  / dt;
      double acc_z = (_cmd_vel->angular.z - vth_) / dt;

      ROS_DEBUG("vel_acc: %f %f %f", acc_x, acc_y, acc_z);

      if (acc_x >   MAX_ACC_X) acc_x =   MAX_ACC_X;
      if (acc_x < - MAX_ACC_X) acc_x = - MAX_ACC_X;
      if (acc_y >   MAX_ACC_Y) acc_y =   MAX_ACC_Y;
      if (acc_y < - MAX_ACC_Y) acc_y = - MAX_ACC_Y;
      if (acc_z >   MAX_ACC_Z) acc_z =   MAX_ACC_Z;
      if (acc_z < - MAX_ACC_Z) acc_z = - MAX_ACC_Z;

      vx_  += acc_x * dt;
      vy_  += acc_y * dt;
      vth_ += acc_z * dt;
    }
*/
    ROS_DEBUG("act_vel: %f %f %f", vx_, vy_, vth_);

    //check servo state
    if ( !servo_on_ ) {
      servo_on_ = true;
      hw_->onWheelServo(servo_on_);
    }

    std::vector<int16_t> wheel_vel(num_of_wheels_);

    // convert velocity to wheel
    // need to declarion check
    velocityToWheel(vx_, vy_, vth_, wheel_vel);

    hw_->turnWheel(wheel_vel);

    //update time_stamp_
    time_stamp_ = now;

    base_mtx_.unlock();
  } else {
    ROS_WARN("cmd_vel comes before sending pervious message");
  }
}

//////////////////////////////////////////////////
/// @brief safety stopper when msg is not reached
///  for more than `safety_duration_` [s]
void robot_hardware::MoverController::safetyCheckCallback(const ros::TimerEvent& _event)
{
  if((ros::Time::now() - time_stamp_).toSec() >= safety_duration_ && servo_on_) {
    std::vector<int16_t> wheel_velocity(num_of_wheels_);
    
    vx_ = vy_ = vth_ = 0.0;
    ROS_WARN("Base: safety stop");
    for (size_t i = 0; i < num_of_wheels_; i++) {
      wheel_velocity[i] = 0;
    }
    hw_->turnWheel(wheel_velocity);

    servo_on_ = false;
    hw_->onWheelServo(servo_on_);
  }
}

//////////////////////////////////////////////////
/// @brief odometry publisher
void robot_hardware::MoverController::calculateOdometry(const ros::TimerEvent& _event)
{
  current_time_ = ros::Time::now();

  double dt, delta_x, delta_y, delta_th;
  dt = (current_time_ - last_time_).toSec();
  delta_x  = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
  delta_y  = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
  delta_th = vth_ * dt;

  x_  += delta_x;
  y_  += delta_y;
  th_ += delta_th;

  // odometry is 6DOF so we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat
      = tf::createQuaternionMsgFromYaw(th_);

  // first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time_;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id  = "base_link";

  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  odom_broadcaster_.sendTransform(odom_trans);

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time_;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x  = vx_;
  odom.twist.twist.linear.y  = vy_;
  odom.twist.twist.angular.z = vth_;

  // publish the message
  odom_pub_.publish(odom);

  last_time_ = current_time_;
}

void robot_hardware::MoverController::velocityToWheel
(double _linear_x, double _linear_y, double _angular_z, std::vector<int16_t>& _wheel_vel) 
{
    float dx, dy, dtheta, theta;
    float v1, v2, v3, v4;
    int16_t front_right_wheel, rear_right_wheel, front_left_wheel, rear_left_wheel;
    theta = 0.0;  // this means angle in local coords, so always 0

    float cos_theta = cos(theta);
    float sin_theta = sin(theta);

    // change dy and dx, because of between ROS and vehicle direction
    dy = (_linear_x * cos_theta - _linear_y * sin_theta);
    dx = (_linear_x * sin_theta + _linear_y * cos_theta);
    dtheta = _angular_z;  // desirede angular velocity

    // calculate wheel velocity
    v1 = k1_ * dtheta + k2_ * ((-cos_theta + sin_theta) * dx + (-cos_theta - sin_theta) * dy);
    v2 = k1_ * dtheta + k2_ * ((-cos_theta - sin_theta) * dx + ( cos_theta - sin_theta) * dy);
    v3 = k1_ * dtheta + k2_ * (( cos_theta - sin_theta) * dx + ( cos_theta + sin_theta) * dy);
    v4 = k1_ * dtheta + k2_ * (( cos_theta + sin_theta) * dx + (-cos_theta + sin_theta) * dy);

    //[rad/sec] -> [deg/sec]
    front_right_wheel = static_cast<int16_t>(v1 * (180 / M_PI));
    rear_right_wheel = static_cast<int16_t>(v4 * (180 / M_PI));
    front_left_wheel = static_cast<int16_t>(v2 * (180 / M_PI));
    rear_left_wheel = static_cast<int16_t>(v3 * (180 / M_PI));

    _wheel_vel[0] = front_left_wheel;
    _wheel_vel[1] = front_right_wheel;
    _wheel_vel[2] = rear_left_wheel;
    _wheel_vel[3] = rear_right_wheel;
}

//////////////////////////////////////////////////
bool robot_hardware::MoverController::setInitialPoseCallback
(seed_r7_ros_controller::SetInitialPose::Request& _req,
seed_r7_ros_controller::SetInitialPose::Response& _res)
{
  geometry_msgs::PoseWithCovarianceStamped initial_pose;
  geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(_req.theta);

  initial_pose.header.stamp = ros::Time::now();
  initial_pose.header.frame_id = "map";
  initial_pose.pose.pose.position.x = _req.x;
  initial_pose.pose.pose.position.y = _req.y;
  initial_pose.pose.pose.position.z = 0.0;
  initial_pose.pose.pose.orientation = pose_quat;
  initialpose_pub_.publish(initial_pose);

  return "SetInitialPose succeeded";
}

bool robot_hardware::MoverController::ledControlCallback
  (seed_r7_ros_controller::LedControl::Request&  _req,
  seed_r7_ros_controller::LedControl::Response& _res)
{
  hw_->runLedScript(_req.send_number, _req.script_number);

  _res.result = "LED succeeded";

  return true;
}