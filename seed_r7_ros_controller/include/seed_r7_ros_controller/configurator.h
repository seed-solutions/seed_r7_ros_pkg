#ifndef _CONFIGURATOR_H_
#define _CONFIGURATOR_H_


#include "seed_r7_ros_controller/seed_r7_robot_hardware.h"
#include "seed_r7_ros_controller/MsSystemConfigWrite.h"
#include "seed_r7_ros_controller/MsSystemConfigRead.h"
#include "seed_r7_ros_controller/MsConfigWrite.h"
#include "seed_r7_ros_controller/MsConfigRead.h"

namespace robot_hardware
{

class Configurator
{
public: 
    Configurator(const ros::NodeHandle& _nh, robot_hardware::RobotHW *_in_hw);
  ~Configurator();

  bool ConfiguratorSysconfigWriteCallback(seed_r7_ros_controller::MsSystemConfigWrite::Request& _req, seed_r7_ros_controller::MsSystemConfigWrite::Response& _res);
  bool ConfiguratorSysconfigReadCallback(seed_r7_ros_controller::MsSystemConfigRead::Request& _req, seed_r7_ros_controller::MsSystemConfigRead::Response& _res);

  bool ConfiguratorConfigWriteCallback(seed_r7_ros_controller::MsConfigWrite::Request& _req, seed_r7_ros_controller::MsConfigWrite::Response& _res);
  bool ConfiguratorConfigReadCallback(seed_r7_ros_controller::MsConfigRead::Request& _req, seed_r7_ros_controller::MsConfigRead::Response& _res);


private: 
  ros::ServiceServer srv_sysconfig_write;
  ros::ServiceServer srv_sysconfig_read;

  ros::ServiceServer srv_config_write;
  ros::ServiceServer srv_config_read;


  robot_hardware::RobotHW *hw_;
  ros::NodeHandle nh_;
};

}

#endif
