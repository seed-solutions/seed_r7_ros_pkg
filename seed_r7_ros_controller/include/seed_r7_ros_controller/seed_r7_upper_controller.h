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

    bool is_open_;
    std::vector<int16_t> raw_data_;

    std::vector<std::string> name_;
    std::vector<int> aero_index_;
    std::vector<std::pair<int,std::string>> aero_table_;

  protected: 
    aero::controller::AeroCommand *upper_;
    const static uint32_t BAUDRATE = 1000000;      
};

}

#endif
