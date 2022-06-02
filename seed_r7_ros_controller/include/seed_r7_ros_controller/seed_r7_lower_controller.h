#ifndef _LOWER_CONTROLLER_H_
#define _LOWER_CONTROLLER_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "seed_smartactuator_sdk/aero3_command.h"


namespace robot_hardware
{

class LowerController
{
  public: 
    LowerController(const std::string& _port);
    ~LowerController();

    void resetting();
    std::vector<uint8_t> read_1byte(uint16_t _address,int size);
    void write_1byte(uint16_t _address, uint8_t *_write_data, int write_size);
    void getPosition();
    void sendPosition(uint16_t _time, std::vector<int16_t>& _data);
    CosmoCmdReqType getCosmoCmd();
    void sendCosmoCmdResp(CosmoCmdRespType resp);
    RobotStatusCmdReqType getRobotStatusCmd();
    void sendRobotStatusCmdResp(RobotStatusCmdRespType resp);
    VirtualControllerCmdReqType getVirtualControllerCmd();
    void remapAeroToRos(std::vector<int16_t>& _ros, std::vector<int16_t>& _aero);
    void remapRosToAero(std::vector<int16_t>& _aero, std::vector<int16_t>& _ros);
    void runScript(uint8_t _number, uint16_t _script);
    void sendVelocity(std::vector<int16_t>& _data);
    void onServo(bool _value);
    float getBatteryVoltage();
    std::string getFirmwareVersion();
    void getRobotStatus(int8_t _number);
    void checkRobotStatus();
    void setJoy(std::vector<uint8_t>& _data);

    bool is_open_;
    std::vector<int16_t> raw_data_;

    std::vector<std::string> upper_name_;
    std::vector<std::string> name_;
    std::vector<int> aero_index_;
    std::vector<std::pair<int,std::string>> aero_table_;

    std::vector<std::string> wheel_name_;
    std::vector<int> wheel_aero_index_;
    std::vector<std::pair<int,std::string>> wheel_table_;

    sensor_msgs::Joy joy_;

    bool comm_err_;
    bool enable_joy_;
    struct RobotStatus {
      bool connection_err_;
      bool calib_err_;
      bool motor_err_;
      bool temp_err_;
      bool res_err_;
      bool step_out_err_;
      bool p_stopped_err_;
      bool power_err_;
    } robot_status_;

  protected:
    aero::controller::AeroCommand *lower_;
    const static uint32_t BAUDRATE = 1000000;

    enum error_bit_t{
      can2_connection = 0,
      can2_calibration = 1,
      can2_motor_status = 2,
      can2_temperature = 3,
      can2_response = 4,
      can2_step_out = 5,
      can2_protective_stopped = 6,
      can2_power = 7,
      can1_connection = 8,
      can1_calibration = 9,
      can1_motor_status = 10,
      can1_temperature = 11,
      can1_response = 12,
      can1_step_out = 13,
      can1_protective_stopped = 14,
      can1_power = 15,
    };
};

}

#endif
