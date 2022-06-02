#pragma once

#include <memory>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>


namespace joint_state_controller{
class JointStateController: public controller_interface::Controller<hardware_interface::JointStateInterface>
{
public:
  JointStateController() : publish_rate_(0.0) {}

  virtual bool init(hardware_interface::JointStateInterface* hw,
                    ros::NodeHandle&                         root_nh,
                    ros::NodeHandle&                         controller_nh);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& period);
  virtual void stopping(const ros::Time& time);

private:
  std::vector<hardware_interface::JointStateHandle> pub_joint_handles;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_;
  ros::Time last_publish_time_;
  double publish_rate_ = 0;
  int all_joints_num = 0;
};
}
