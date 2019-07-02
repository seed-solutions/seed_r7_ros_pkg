#ifndef _ROBOT_CONTROLLER_H_
#define _ROBOT_CONTROLLER_H_

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <stdint.h>
#include <unistd.h>
#include <unordered_map>
#include <cmath>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

//#include "seed_solutions_sdk"
#include "AJointIndex.hh"
#include "noid_typef_description/robots/headers/Constants.hh"

namespace noid
{
    namespace controller
    {
        class NoidControllerProto
        {
           
        public:
        NoidController(const std::string& _port, uint8_t _id);
        ~NoidController();

        void get_data(std::vector<int16_t>& _stroke_vector);

        void get_command(uint8_t _cmd, uint8_t _sub, std::vector<int16_t>& _stroke_vector);
        
        bool get_status();

        std::vector<int16_t> get_actual_stroke_vector();
        int get_number_of_angle_joints();
        std::string get_stroke_joint_name(size_t _idx);
        int get_number_of_strokes();
        bool get_joint_name(int32_t _joint_id, std::string &_name);
        void update_position();

        void servo_command(int16_t _d0, int16_t _d1);

        /// @brief set position command (waiting return of current position)
        /// @param _stroke_vector stroke vector, MUST be DOF bytes
        /// @param _time time[ms]
        void set_position(std::vector<int16_t>& _stroke_vector,
                               uint16_t _time);

        protected: 
        std::vector<int16_t> stroke_cur_vector_;
        std::vector<AJointIndex> stroke_joint_indices_;
        std::unordered_map<std::string, int32_t> angle_joint_indices_;
        boost::mutex ctrl_mtx_;


        };

        class NoidUpperController : public NoidControllerProto
        {
        public: 
        NoidUpperController(const std::string& _port);

        ~NoidUpperController();

        };
        /// @brief Lower body controller
        ///
        /// This class has some functions related to wheel control
        class NoidLowerController : public NoidControllerProto
        {
        public:
        NoidLowerController(const std::string& _port);
        ~NoidLowerController(); 
        /// @brief servo toggle command with wheels
        /// @param _d0 joints 1: on, 0: off
        /// @param _d1 wheels 1: on, 0: off
        void servo_command(int16_t _d0, int16_t _d1);

        /// @brief set wheel velocity
        void set_wheel_velocity(std::vector<int16_t>& _wheel_vector, uint16_t _time);        
        std::string get_wheel_name(size_t _idx);
        std::vector<int16_t>& get_reference_wheel_vector();
        int32_t get_wheel_id(std::string& _name);
        void startWheelServo();
        void stopWheelServo();

        //BaseController library
        void VelocityToWheel(double _linear_x, double _linear_y, double _angular_z,
                           std::vector<int16_t>& _wheel_vel);
        
        void writeWheel(const std::vector< std::string> &_names, const std::vector<int16_t> &_vel, double _tm_sec);

        protected:
        std::vector<int16_t> wheel_vector_;
        std::vector<int16_t> wheel_ref_vector_;
        std::vector<int16_t> wheel_cur_vector_;

        std::vector<AJointIndex> wheel_indices_;

        bool wheel_servo_;

        };

        /* ---- Move to CommandList.hh ----------*/
        // header offset = 5bytes
        const static size_t RAW_HEADER_OFFSET = 5;
        // data length = 68bytes
        const static size_t RAW_DATA_LENGTH = 68;
        
        /* ---- Move to AeroControllers.hh ------*/
        int16_t decode_short_(uint8_t* _raw);
         /// @brief ecnode short(int16_t) to byte(uint8_t)
        void encode_short_(int16_t _value, uint8_t* _raw);
    }
}