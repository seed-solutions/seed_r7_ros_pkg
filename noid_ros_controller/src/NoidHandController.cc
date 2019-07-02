#include <ros/ros.h>
#include <HandControl.h>
#include <GraspControl.h>

#include "NoidHandController.hh" 


class NoidHandInterface : public robot_interface::RobotInterface
{

using namespace noid_ros_controller;

class NoidHandControl {
public:
  NoidHandControl (ros::NodeHandle &nh) : executing_flg_left_(true), executing_flg_right_(true), exist_grasp_server_(false)
  {
    hi.reset(new NoidHandInterface(nh));

    service_ = nh.advertiseService("/noid_hand_controller",
                                   &NoidHandControl::HandControl, this);

    g_client_ = nh.serviceClient<noid_ros_controller::GraspControl>(
      "/noid_ros_controller/grasp_control");

    if ( g_client_.waitForExistence(ros::Duration(3.0)) ) {
      exist_grasp_server_ = true;
    } else {
      ROS_WARN("GraspServer not found");
    }

    {
      noid_ros_conotroller::GraspControl g_srv;
      g_srv.request.position = POSITION_Left;
      g_srv.request.script = GraspControlRequest::SCRIPT_CANCEL;
      g_srv.request.power = (100 << 8) + 30;
      ROS_DEBUG("call pos: %d, script: %d, power %d",
                g_srv.request.position, g_srv.request.script, g_srv.request.power);
      if (exist_grasp_server_) {
        g_client_.call(g_srv);
      }
      executing_flg_left_ = false;
    }
    {
      noid_ros_controller::GraspControl g_srv;
      g_srv.request.position = POSITION_Right;
      g_srv.request.script = GraspControlRequest::SCRIPT_CANCEL;
      g_srv.request.power = (100 << 8) + 30;
      ROS_DEBUG("call pos: %d, script: %d, power %d",
                g_srv.request.position, g_srv.request.script, g_srv.request.power);
      if (exist_grasp_server_) {
        g_client_.call(g_srv);
      }
      executing_flg_right_ = false;
    }
    ROS_INFO("Initialized Handcontroller");
  }

  bool HandControl(noid_ros_controller::HandControl::Request &req,
                   noid_ros_controller::HandControl::Response &res)
  {
    ROS_DEBUG("HandControl com: %d, hand %d, pow: %d, time: %f, thre: %f %f, lr_ang: %f %f",
              req.command, req.hand, req.power, req.time_sec,
              req.thre_fail, req.thre_warn, req.larm_angle, req.rarm_angle);

    if (req.hand != HandControlRequest::HAND_LEFT &&
        req.hand != HandControlRequest::HAND_RIGHT &&
        req.hand != HandControlRequest::HAND_BOTH ) {
      ROS_ERROR("not existing hand (= %d) for aero_startup/HandControl service", req.hand);
      res.success = false;
      res.status  = "invalid hand parameter";
      return true;
    }

    noid_ros_controller::GraspControl g_srv;

    int power = req.power;
    float grasp_time  = 1.0;
    if (req.time_sec > 0) {
      grasp_time = req.time_sec;
    }
    res.success = true; // substitude 'false' if needed
    res.status = "grasp check not supported, always return success";

    switch (req.command) {
    case HandControlRequest::COMMAND_GRASP:
      {
        if (req.hand == HandControlRequest::HAND_LEFT) {
          g_srv.request.position = POSITION_Left;
          g_srv.request.script = GraspControlRequest::SCRIPT_GRASP;
          executing_flg_left_ = true; //executing_grasp_script
        } else if (req.hand == HandControlRequest::HAND_RIGHT) {
          g_srv.request.position = POSITION_Right;
          g_srv.request.script = GraspControlRequest::SCRIPT_GRASP;
          executing_flg_right_ = true; //executing_grasp_script
        } else {
          ROS_ERROR("Unexpected hand %d", req.hand);
          return false;
        }

        if (power != 0) {
          g_srv.request.power = (power << 8) + power;
        } else {
          g_srv.request.power = (100 << 8) + 100;
        }
        ROS_DEBUG("call pos: %d, script: %d, power %d",
                  g_srv.request.position, g_srv.request.script, g_srv.request.power);
        if (exist_grasp_server_) {
          g_client_.call(g_srv);
        }

      }
      break;
    case HandControlRequest::COMMAND_UNGRASP:
      {
        OpenHand(req.hand); // applying time may cause step out, handle with script
        res.status = "ungrasp success";
      }
      break;
    #if 0
    case HandControlRequest::COMMAND_GRASP_ANGLE:
      {
	// TODO: return fail if step-out was detected
        GraspAngle(req.hand, req.larm_angle, req.rarm_angle, grasp_time);
        res.status = "grasp-angle success";
      }
      break;
    #endif
    default:
      ROS_ERROR("command does not exist for aero_startup/HandControl service");
    }
    return true;
  }

  void OpenHand(int hand)
  {
    ROS_DEBUG("OpenHand %d", hand);
    noid_ros_controller::GraspControl g_srv;

    robot_interface::joint_angle_map map;
    if (hand == HandControlRequest::HAND_LEFT) {
      g_srv.request.position = POSITION_Left;
      g_srv.request.script = GraspControlRequest::SCRIPT_UNGRASP;
      executing_flg_left_ = false;
      // L_OPEN();
    } else if (hand == HandControlRequest::HAND_RIGHT) {
      g_srv.request.position = POSITION_Right;
      g_srv.request.script = GraspControlRequest::SCRIPT_UNGRASP;
      executing_flg_right_ = false;
      // R_OPEN();
    }
    // g_srv.request.power = (100 << 8) + 30; // no meaning
    ROS_DEBUG("call pos: %d, script: %d",
              g_srv.request.position, g_srv.request.script);
    if (exist_grasp_server_) {
      g_client_.call(g_srv);
    }
  }
#if 0
  void GraspAngle (int hand, float larm_angle, float rarm_angle, float time=0.5)
  {
    ROS_DEBUG("Grasp Angle: %d %f %f %f", hand, larm_angle, rarm_angle, time);
    //aero_startup::AeroSendJoints srv;
    robot_interface::joint_angle_map map;
    aero_startup::GraspControl g_srv;
    switch(hand) {
    case HandControlRequest::HAND_BOTH:
      R_GRASP();
      L_GRASP();
      break;

    case HandControlRequest::HAND_LEFT:
      g_srv.request.position = POSITION_Left;
      if (executing_flg_left_) {
        g_srv.request.script = GraspControlRequest::SCRIPT_CANCEL;
        g_srv.request.power = (100 << 8) + 30;
        ROS_DEBUG("call pos: %d, script: %d, power %d",
                  g_srv.request.position, g_srv.request.script, g_srv.request.power);
        if (exist_grasp_server_) {
          g_client_.call(g_srv);
        }
        executing_flg_left_ = false;
      }
      L_GRASP ();
      break;

    case HandControlRequest::HAND_RIGHT:
      g_srv.request.position = POSITION_Right;
      if (executing_flg_right_) {
        g_srv.request.script = GraspControlRequest::SCRIPT_CANCEL;
        g_srv.request.power = (100 << 8) + 30;
        ROS_DEBUG("call pos: %d, script: %d, power %d",
                  g_srv.request.position, g_srv.request.script, g_srv.request.power);
        if (exist_grasp_server_) {
          g_client_.call(g_srv);
        }
        executing_flg_right_ = false;
      }
      R_GRASP ();
      break;
    }
    ROS_DEBUG("GraspAngle: sendAngles");
    ros::Time start = ros::Time::now() + ros::Duration(0.04);
    hi->sendAngles(map, time, start);
    ROS_DEBUG("GraspAngle: wait_interpolation");
    hi->wait_interpolation();
    usleep(50*1000); // sleep 50ms for waiting to finish position command
    g_srv.request.script = GraspControlRequest::COMMAND_SERVO; // in case step-out
    if (exist_grasp_server_) {
      g_client_.call(g_srv);
    }
  }
#endif

private:
  bool executing_flg_left_;
  bool executing_flg_right_;
  bool exist_grasp_server_;

  ros::ServiceClient g_client_;
  ros::ServiceServer service_;

  boost::shared_ptr<NoidHandInterface > hi;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aero_hand_controller");
  ros::NodeHandle nh;
  NoidHandControl ahc(nh);

  ros::spin();

  return 0;
}
