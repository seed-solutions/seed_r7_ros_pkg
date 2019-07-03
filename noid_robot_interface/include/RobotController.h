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
#include <mutex>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

//#include "seed_solutions_sdk"
#include "AJointIndex.h"


namespace noid
{
    namespace controller
    {
        class NoidControllerProto
        {
           
        public:
        NoidControllerProto(const std::string& _port, uint8_t _id);
        ~NoidControllerProto();

        void get_data(std::vector<int16_t>& _stroke_vector);

        void get_command(uint8_t _cmd, uint8_t _sub, std::vector<int16_t>& _stroke_vector);
        void get_command(uint8_t _cmd, std::vector<int16_t>& _stroke_vector);
        bool get_status();
        void reset_status();


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

        /// @brief stoke_vector to raw command bytes
        void stroke_to_raw_(std::vector<int16_t>& _stroke, std::vector<uint8_t>& _raw);

        protected: 
        std::vector<int16_t> stroke_cur_vector_;
        std::vector<int16_t> stroke_ref_vector_;

        std::vector<AJointIndex> stroke_joint_indices_;
        std::unordered_map<std::string, int32_t> angle_joint_indices_;
        boost::mutex ctrl_mtx_;
        bool bad_status_;
        std::vector<int16_t> status_vector_;



        // id: upper = 1, lower = 2
        const static uint8_t ID_UPPER = 1;
        const static uint8_t ID_LOWER = 2;

        // whole: 34DOF
        //  upper: 22DOF
        //   neck: 3DOF
        //   arm: 8DOF (including hand) * 2
        //    shoulder: 2DOF * 2
        //    elbow: 2DOF * 2
        //    wrist: 3DOF * 2
        //    hand: 1DOF * 2
        //   waist: 3DOF
        //  lower : 0DOF
        //    (wheel: 1DOF * 4)

        const static size_t AERO_DOF = 24;
        const static size_t AERO_DOF_UPPER = 22;
        const static size_t AERO_DOF_LOWER = 2;
        const static size_t AERO_DOF_WHEEL = 4;

        // joint index in stroke vector
        // UPPER:
        const static size_t CAN_NECK_Y = 0;
        const static size_t CAN_NECK_RIGHT = 1;
        const static size_t CAN_NECK_LEFT = 2;

        const static size_t CAN_R_SHOULDER_P = 3;
        const static size_t CAN_R_SHOULDER_R = 4;
        const static size_t CAN_R_ELBOW_Y = 5;
        const static size_t CAN_R_ELBOW_P = 6;
        const static size_t CAN_R_WRIST_Y = 7;
        const static size_t CAN_R_WRIST_TOP = 8;
        const static size_t CAN_R_WRIST_BOTTOM = 9;
        const static size_t CAN_R_HAND = 10;

        const static size_t CAN_L_SHOULDER_P = 11;
        const static size_t CAN_L_SHOULDER_R = 12;
        const static size_t CAN_L_ELBOW_Y = 13;
        const static size_t CAN_L_ELBOW_P = 14;
        const static size_t CAN_L_WRIST_Y = 15;
        const static size_t CAN_L_WRIST_TOP = 16;
        const static size_t CAN_L_WRIST_BOTTOM = 17;
        const static size_t CAN_L_HAND = 18;

        const static size_t CAN_WAIST_RIGHT = 19;
        const static size_t CAN_WAIST_LEFT = 20;
        const static size_t CAN_WAIST_Y = 21;

         // LOWER:
        const static size_t CAN_DOWN = 0;
        const static size_t CAN_UP = 1;

        // WHEEL:
        const static size_t CAN_FRONT_R_WHEEL = 0;
        const static size_t CAN_REAR_R_WHEEL = 1;
        const static size_t CAN_FRONT_L_WHEEL = 2;
        const static size_t CAN_REAR_L_WHEEL = 3;


        // joint index in raw vector (as int16_t)
        // UPPER: ID = 1
        const static size_t RAW_NECK_Y = 0;
        const static size_t RAW_NECK_RIGHT = 1;
        const static size_t RAW_NECK_LEFT = 2;
        const static size_t RAW_R_SHOULDER_P = 3;
        const static size_t RAW_R_SHOULDER_R = 4;
        const static size_t RAW_R_ELBOW_Y = 5;
        const static size_t RAW_R_ELBOW_P = 6;
        const static size_t RAW_R_WRIST_Y = 7;
        const static size_t RAW_R_WRIST_TOP = 8;
        const static size_t RAW_R_WRIST_BOTTOM = 9;
        const static size_t RAW_WAIST_RIGHT = 10;
        const static size_t RAW_R_HAND = 11;
        // 12 - 15: Force Sensor (uint8_t * 6, 2bytes N/A)
        const static size_t RAW_WAIST_Y = 15;
        // 17 - 18: N/A
        const static size_t RAW_L_SHOULDER_P = 18;
        const static size_t RAW_L_SHOULDER_R = 19;
        const static size_t RAW_L_ELBOW_Y = 20;
        const static size_t RAW_L_ELBOW_P = 21;
        const static size_t RAW_L_WRIST_Y = 22;
        const static size_t RAW_L_WRIST_TOP = 23;
        const static size_t RAW_L_WRIST_BOTTOM = 24;
        const static size_t RAW_WAIST_LEFT = 25;
        const static size_t RAW_L_HAND = 26;
        // 28 - 31: Force Sensor (uint8_t * 6, 2bytes N/A)
        // 32 - 34: N/A

        // LOWER :
        const static size_t RAW_DOWN = 0;
        const static size_t RAW_UP = 1;
        const static size_t RAW_FRONT_R_WHEEL = 2;
        const static size_t RAW_REAR_R_WHEEL = 3;
        // 12 - 15: N/A
        const static size_t RAW_FRONT_L_WHEEL = 4;
        const static size_t RAW_REAR_L_WHEEL = 5;
        // 28 - 31: N/A
        // 32 - 34: IMU (uint8_t * 6)

        // offsets
        // UPPER:
        const static size_t OFFSET_R_SHOULDER_R = 1119;
        //const static size_t OFFSET_R_HAND = 900;
        const static size_t OFFSET_L_SHOULDER_R = 1119;
        //const static size_t OFFSET_L_HAND = 900;

        // LOWER:

        // sensor index (as int8_t)
        // UPPER:
        const static size_t RIGHT_HAND_SENSOR_FX = 30;
        const static size_t RIGHT_HAND_SENSOR_FY = 31;
        const static size_t RIGHT_HAND_SENSOR_FZ = 32;
        const static size_t RIGHT_HAND_SENSOR_RX = 33;
        const static size_t RIGHT_HAND_SENSOR_RY = 34;
        const static size_t RIGHT_HAND_SENSOR_RZ = 35;

        const static size_t LEFT_HAND_SENSOR_FX = 62;
        const static size_t LEFT_HAND_SENSOR_FY = 63;
        const static size_t LEFT_HAND_SENSOR_FZ = 64;
        const static size_t LEFT_HAND_SENSOR_RX = 65;
        const static size_t LEFT_HAND_SENSOR_RY = 66;
        const static size_t LEFT_HAND_SENSOR_RZ = 67;

        // LOWER :



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
        std::mutex mutex_lower_;


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

#endif