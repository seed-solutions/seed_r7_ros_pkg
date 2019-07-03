#ifndef _NOID_UPPER_CONTROLLER_H_
#define _NOID_UPPER_CONTROLLER_H_

#include <ros/ros.h>
#include "seed_solutions_sdk/aero3_command.h"

namespace noid
{
  namespace controller
  {
    class NoidUpperController 
    {
      public: NoidUpperController(const std::string& _port);
      public: ~NoidUpperController();

      public: 
        void getPosition();
        void sendPosition(uint16_t _time, std::vector<int16_t>& _data);
        void remapAeroToRos(std::vector<int16_t>& _before, std::vector<int16_t>& _after);
        void remapRosToAero(std::vector<int16_t>& _before, std::vector<int16_t>& _after);

        bool is_open_;
        std::vector<int16_t> raw_data_;
        unsigned int number_of_angles_;

        std::vector<std::string> name_;
        std::vector<int> aero_index_;
        std::vector<int> ros_index_;
        int DOF_;
        
      protected: 
        aero::controller::AeroCommand *upper_;
        const static uint32_t BAUDRATE = 1000000;
      
    };

  }
}

#endif
