#include <ros/ros.h>
<<<<<<< HEAD
#include <noid_ros_controller/HandControl.h>
#include <noid_ros_controller/GraspControl.h>

#include <noid_grasp.h>

using namespace noid_ros_controller;

class NoidHandControl {
public:
  NoidHandControl (ros::NodeHandle &nh) : executing_flg_left_(true), executing_flg_right_(true), exist_grasp_server_(false)
  {

    service_ = nh.advertiseService("/noid_hand_controller",
                                   &NoidHandControl::HandControl, this);

    g_client_ = nh.serviceClient<noid_ros_controller::GraspControl>(
      "/noid_ros_controller/grasp_control");

    nh.getParam("noid_hand_controller/POSITION_Right", hand_right_num_);
    nh.getParam("noid_hand_controller/POSITION_Left", hand_left_num_);

    if ( g_client_.waitForExistence(ros::Duration(3.0)) ) {
      exist_grasp_server_ = true;
    } else {
      ROS_WARN("GraspServer not found");
    }

    {
      noid_ros_controller::GraspControl g_srv;
      g_srv.request.position = hand_left_num_;
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
      g_srv.request.position = hand_right_num_;
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
      ROS_ERROR("not existing hand (= %d) for noid_ros_controller/HandControl service", req.hand);
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
          g_srv.request.position = hand_left_num_;
          g_srv.request.script = GraspControlRequest::SCRIPT_GRASP;
          executing_flg_left_ = true; //executing_grasp_script
        } else if (req.hand == HandControlRequest::HAND_RIGHT) {
          g_srv.request.position = hand_right_num_;
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
        ROS_INFO("call pos: %d, script: %d, power %d",
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
      ROS_ERROR("command does not exist for noid_ros_controller/HandControl service");
    }
    return true;
  }

  void OpenHand(int hand)
  {
    ROS_DEBUG("OpenHand %d", hand);
    noid_ros_controller::GraspControl g_srv;

    if (hand == HandControlRequest::HAND_LEFT) {
      g_srv.request.position = hand_left_num_;
      g_srv.request.script = GraspControlRequest::SCRIPT_UNGRASP;
      executing_flg_left_ = false;
      // L_OPEN();
    } else if (hand == HandControlRequest::HAND_RIGHT) {
      g_srv.request.position = hand_right_num_;
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

  int hand_right_num_;
  int hand_left_num_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "noid_hand_controller");
  ros::NodeHandle nh;
  NoidHandControl ahc(nh);

  ros::spin();

  return 0;
}
=======
#include <noid_ros_controller/GraspControl.h>

#include <noid_hand_controller.h>

using namespace noid_ros_controller;
using namespace noid;
using namespace grasp;


NoidHandControl::NoidHandControl(const ros::NodeHandle& _nh, 
                                noid_robot_hardware::NoidRobotHW *_in_hw):hand_right_num_(0),hand_left_num_(0)
{
  ROS_INFO("grasp_server_start");

  hw_ = _in_hw;
  nh_ = _nh;
  grasp_control_server_ = nh_.advertiseService("hand_control",
                                                &NoidHandControl::GraspControlCallback,this);

  if(nh_.hasParam("/noid_hand_controller/right_hand")) 
  {
    nh_.getParam("/noid_hand_controller/right_hand", hand_right_num_);
  }
  if(nh_.hasParam("/noid_hand_controller/left_hand")) 
  {
    nh_.getParam("/noid_hand_controller/left_hand", hand_left_num_);
  }

  //initialize script cancel on right_hand
  hw_->handScript(hand_right_num_, script_cancel);
  //initialize script cancel on left_hand
  hw_->handScript(hand_left_num_, script_cancel);
    
  ROS_INFO("Initialized Handcontroller");
}   

NoidHandControl::~NoidHandControl()
{

}

bool NoidHandControl::GraspControlCallback(noid_ros_controller::GraspControl::Request&  _req,
                                   noid_ros_controller::GraspControl::Response& _res) 
{
   ROS_INFO("Grasp callback start");
   
   if(_req.position == "right")
   {
           // return if cancel script
      if (_req.script == "cancel") {
        SelectHandScript(_req.script, hand_right_num_ ,script_cancel, _req.power);
      } 
      else if (_req.script == "grasp") {
        SelectHandScript(_req.script, hand_right_num_ ,script_grasp, _req.power);
        ros::Duration(2.8).sleep(); // wait 2.8 seconds, as script takes max 2.8 seconds!
      } 
      else if (_req.script == "ungrasp") {
        SelectHandScript(_req.script, hand_right_num_ ,script_ungrasp, _req.power);
        ros::Duration(2.7).sleep(); // wait 2.7 seconds, as script takes max 2.7 seconds!
      }
      else
      {
        ROS_ERROR("please input string_type or valid name");
        _res.result = "service call failed";
      }
   }
   else if(_req.position == "left")
   {
      // return if cancel script
      if (_req.script == "cancel") {
        SelectHandScript(_req.script, hand_left_num_ ,script_cancel, _req.power);
      } else if (_req.script == "grasp") {
        SelectHandScript(_req.script, hand_left_num_ ,script_grasp, _req.power);
        ros::Duration(2.8).sleep(); // wait 2.8 seconds, as script takes max 2.8 seconds!
      } else if (_req.script == "ungrasp") {
        SelectHandScript(_req.script, hand_left_num_ ,script_ungrasp, _req.power);
        ros::Duration(2.7).sleep(); // wait 2.7 seconds, as script takes max 2.7 seconds!
      }
      else
      {
        ROS_ERROR("please input string_type or valid name");
        _res.result = "service call failed";
      }
   }
   ROS_INFO("End Grasp");
   return true;
   
 
}

void NoidHandControl::SelectHandScript(std::string _motion, int16_t _hand_type, int16_t _hand_scriptnum, int16_t _power)
{
      ROS_INFO("handscript: setMaxSingleCurrent");
      //hw_->setMaxSingleCurrent(_hand_type, _power);
      ROS_INFO("motion: %s", _motion.c_str());
      hw_->handScript(_hand_type, _hand_scriptnum);
  
}



>>>>>>> 2f195b7a53d7118c7ec6551d133377636a98c6b4
