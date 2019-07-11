#ifndef _NOID_LOWER_CONTROLLER_H_
#define _NOID_LOWER_CONTROLLER_H_

#include <ros/ros.h>
#include "seed_solutions_sdk/aero3_command.h"

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
        void remapAeroToRos(std::vector<int16_t>& _before, std::vector<int16_t>& _after);
        void remapRosToAero(std::vector<int16_t>& _before, std::vector<int16_t>& _after);

        void sendVelocity(uint16_t _time, std::vector<int16_t>& _data);

        void servo_command(int16_t _d1);
        void encode_short_(int16_t _value, uint8_t* _raw);



        bool is_open_;
        std::vector<int16_t> raw_data_;
        unsigned int number_of_angles_;
        unsigned int number_of_wheels_;

        std::vector<std::string> name_;
        std::vector<int> aero_index_;
        std::vector<int> ros_index_;
        int DOF_;
        int DOF_wheel_;

      protected:
        aero::controller::AeroCommand *lower_;
        const static uint32_t BAUDRATE = 1000000;   
    };

  }
}

#endif
