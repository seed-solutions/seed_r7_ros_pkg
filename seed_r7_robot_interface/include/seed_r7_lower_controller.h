#ifndef _NOID_LOWER_CONTROLLER_H_
#define _NOID_LOWER_CONTROLLER_H_

#include <ros/ros.h>
#include "seed_smartactuator_sdk/aero3_command.h"

namespace noid
{
  namespace controller
  {    
    class NoidLowerController
    {
      public: NoidLowerController(const std::string& _port);
      public: ~NoidLowerController();

      public: 
        void getPosition();
        void sendPosition(uint16_t _time, std::vector<int16_t>& _data);
        void remapAeroToRos(std::vector<int16_t>& _ros, std::vector<int16_t>& _aero);
        void remapRosToAero(std::vector<int16_t>& _aero, std::vector<int16_t>& _ros);
        void sendVelocity(std::vector<int16_t>& _data);
        void onServo(bool _value);

        bool is_open_;
        std::vector<int16_t> raw_data_;

        std::vector<std::string> upper_name_;
        std::vector<std::string> name_;
        std::vector<int> aero_index_;
        std::vector<std::pair<int,std::string>> aero_table_;

        std::vector<std::string> wheel_name_;
        std::vector<int> wheel_aero_index_;
        std::vector<std::pair<int,std::string>> wheel_table_;

      protected:
        aero::controller::AeroCommand *lower_;
        const static uint32_t BAUDRATE = 1000000;   
    };

  }
}

#endif
