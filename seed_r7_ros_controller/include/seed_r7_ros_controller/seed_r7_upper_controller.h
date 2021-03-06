#ifndef _UPPER_CONTROLLER_H_
#define _UPPER_CONTROLLER_H_

#include <ros/ros.h>
#include "seed_smartactuator_sdk/aero3_command.h"


namespace robot_hardware
{

class UpperController 
{
  public:
    UpperController(const std::string& _port);
    ~UpperController();

    void getPosition();
    void sendPosition(uint16_t _time, std::vector<int16_t>& _data);
    void remapAeroToRos(std::vector<int16_t>& _ros, std::vector<int16_t>& _aero);
    void remapRosToAero(std::vector<int16_t>& _aero, std::vector<int16_t>& _ros);
    void setCurrent(uint8_t _number, uint8_t _max, uint8_t _down);
    void runScript(uint8_t _number, uint16_t _script);
    std::string getFirmwareVersion();
    void checkRobotStatus();

    bool is_open_;
    std::vector<int16_t> raw_data_;

    std::vector<std::string> name_;
    std::vector<int> aero_index_;
    std::vector<std::pair<int,std::string>> aero_table_;

    bool comm_err_;
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
    aero::controller::AeroCommand *upper_;
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
