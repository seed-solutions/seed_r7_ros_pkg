#include <algorithm>
#include <cstddef>
#include <pluginlib/class_list_macros.hpp>

#include "joint_state_controller.hpp"

namespace joint_state_controller {

bool JointStateController::init(hardware_interface::JointStateInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
    std::vector < std::string > jnames;
    std::vector < std::string > jnames_notpub;

    controller_nh.getParam("joints_not_publish", jnames_notpub);
    jnames = hw->getNames();

    all_joints_num = jnames.size();

    if (!controller_nh.getParam("publish_rate", publish_rate_)) {
        ROS_ERROR("publish_rate must be set");
        return false;
    }

    realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "joint_states", 4));

    for (int idx = 0; idx < all_joints_num; idx++) {
        if(std::find(jnames_notpub.begin(),jnames_notpub.end(),jnames[idx]) == jnames_notpub.end()){
            pub_joint_handles.push_back(hw->getHandle(jnames[idx]));
            realtime_pub_->msg_.name.push_back(jnames[idx]);
            realtime_pub_->msg_.position.push_back(0.0);
            realtime_pub_->msg_.velocity.push_back(0.0);
            realtime_pub_->msg_.effort.push_back(0.0);
        }
    }

    return true;
}

void JointStateController::starting(const ros::Time &time) {
    last_publish_time_ = time;
}

void JointStateController::update(const ros::Time &time, const ros::Duration& period) {
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {

        if (realtime_pub_->trylock()) {

            realtime_pub_->msg_.header.stamp = time;
            for (size_t idx = 0; idx < pub_joint_handles.size(); idx++) {
                realtime_pub_->msg_.position[idx] = pub_joint_handles[idx].getPosition();
                realtime_pub_->msg_.velocity[idx] = pub_joint_handles[idx].getVelocity();
                realtime_pub_->msg_.effort[idx] = pub_joint_handles[idx].getEffort();
            }
            realtime_pub_->unlockAndPublish();
            last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);
        }
    }
}

void JointStateController::stopping(const ros::Time& /*time*/) {
}

}

PLUGINLIB_EXPORT_CLASS(joint_state_controller::JointStateController, controller_interface::ControllerBase)
