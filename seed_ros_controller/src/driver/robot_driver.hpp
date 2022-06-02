#pragma once

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <pluginlib/class_loader.h>

#include "stroke_converter_base.hpp"
#include "robot_driver_single_usb.hpp"

#include "other_cmd_buff.hpp"

#include "robot_status.hpp"

class RobotDriver
{
public:
    RobotDriver(ros::NodeHandle &root_nh,ros::NodeHandle &my_nh) : converter_loader_("seed_ros_controller", "seed_converter::StrokeConverter") {
        XmlRpc::XmlRpcValue usb_settings;
        XmlRpc::XmlRpcValue ms_settings;

        root_nh.getParam("usb_settings",usb_settings);
        root_nh.getParam("ms_settings",ms_settings);

        std::vector<int> ms_id_checker;
        for (int usb_idx = 0;usb_idx < usb_settings.size();++usb_idx) {
            auto param = usb_settings[usb_idx];
            if (!param["port"].valid() || !param["mslist"].valid()) {
                continue;
            }

            auto usb_port = std::string(param["port"]);
            int baudrate = 1000000;
            if(param["baudrate"].valid()){
                baudrate = int(param["baudrate"]);
            }

            RobotDriverSingleUSB *usbDriver = new RobotDriverSingleUSB(usb_port, baudrate);

            XmlRpc::XmlRpcValue  mslist = param["mslist"];
            for(int ms_idx = 0; ms_idx < mslist.size();++ms_idx){
                auto key = std::string(mslist[ms_idx]);
                if(ms_settings[key].valid()){
                    auto ms_driver = new MSDriver(ms_settings[key]);

                    //msidの重複をチェックしておく
                    if (ms_id_checker.end() != std::find(ms_id_checker.begin(), ms_id_checker.end(), ms_driver->getMsId())) {
                        std::stringstream msg;
                        msg << "Bad configure of ms_id in driver settings. The msid ("<< ms_driver->getMsId()<<") is duplicated.";
                        throw std::runtime_error(msg.str());
                    }
                    ms_id_checker.push_back(ms_driver->getMsId());

                    usbDriver->addMsDriver(ms_driver);
                }
            }
            usbs.push_back(usbDriver);
        }

        std::string robot_model_plugin_name;
        if (my_nh.getParam("robot_model_plugin", robot_model_plugin_name)) {
            try {
                stroke_converter_ = converter_loader_.createInstance(robot_model_plugin_name);
                stroke_converter_->initialize(my_nh);
                stroke_converter_->setJointNames(getJointNames());
            } catch (pluginlib::PluginlibException &ex) {
                ROS_ERROR("The plugin failed to load : %s", ex.what());
            }
        } else {
            ROS_ERROR_STREAM("robot_model_plugin param is not set");
        }

        //ストロークと関節数は一対一で対応するものとする。
        strokes.resize(getNumJoints());
        positions_fb.resize(getNumJoints());
        joint_position_fb.resize(getNumJoints());
        joint_position_fb_prev1.resize(getNumJoints());
        joint_position_fb_prev2.resize(getNumJoints());

        actuator_vel.resize(getNumJoints());
        joint_velocity.resize(getNumJoints());
        joint_velocity_fb.resize(getNumJoints());

    }

    ~RobotDriver(){
        auto itr = usbs.begin();
        while(itr != usbs.end()){
            auto tmp = *itr;
            itr = usbs.erase(itr);
            delete tmp;
        }
    }

    int getNumPorts(){
        return usbs.size();
    }

    int getNumMs(){
        int num = 0;
        for (auto &usb : usbs) {
            num += usb->getNumMs();
        }
        return num;
    }

    int getNumJoints(){
        int num = 0;
        for (auto &usb : usbs) {
            num += usb->getNumJoints();
        }
        return num;
    }

    int getMsId(int idx){
        for (auto &usb : usbs) {
            if (idx >= usb->getNumMs()) {
                idx -= usb->getNumMs();
            } else {
                return usb->getMsId(idx);
            }
        }
        return -1;//解決できない場合
    }

    template<class T>
    void copy(std::vector<T> &lhs,const std::vector<T> &rhs){
        size_t size = lhs.size() < rhs.size() ? lhs.size():rhs.size();
        size *= sizeof(T);
        memcpy(lhs.data(),rhs.data(),size);
    }

    void calcVelocity(const ros::Time &time, const ros::Duration &period) {
        for (size_t idx = 0; idx < joint_velocity_fb.size(); ++idx) {

            auto diff = joint_position_fb[idx] - joint_position_fb_prev1[idx];
            if(diff < -M_PI){
                diff += 2*M_PI;
            }else if(diff > M_PI){
                diff -= 2*M_PI;
            }

            joint_velocity_fb[idx] = diff / period.toSec();
        }
    }

    void read(const ros::Time &time, const ros::Duration &period){
        for(auto &usb:usbs){
            usb->read();
        }

        int ofst = 0;
        for(auto &usb:usbs){
            usb->getPosition(&positions_fb[ofst]);
            ofst += usb->getNumJoints();
        }

        copy(joint_position_fb_prev2,joint_position_fb_prev1);
        copy(joint_position_fb_prev1,joint_position_fb);
        stroke_converter_->Stroke2Angle(joint_position_fb, positions_fb);

        calcVelocity(time,period);
    }


    void getPos(std::vector<double>& joint_position_){
        if(!stroke_converter_){
            return;
        }

        copy(joint_position_,joint_position_fb);
    }

    void getVel(std::vector<double>& joint_velocity_){
        if(!stroke_converter_){
            return;
        }
        copy(joint_velocity_,joint_velocity_fb);
    }


    void getStatus(Status& status){
        int ofst = 0;
        for(size_t idx = 0;idx < usbs.size();++idx){
            ofst += usbs[idx]->getMsStatus(status.ms_status,ofst);
            usbs[idx]->getUSBStatus(status.usb_status[idx]);
        }

    }


    void sendPosition(const std::vector<double>& command, const double &tgt_time_sec){
        if(!stroke_converter_){
            return;
        }

        // 指令値をストロークに変換する
        stroke_converter_->Angle2Stroke(strokes, command);

        //各USBで送信
        int ofst = 0;
        for(auto &usb:usbs){
            usb->sendPosition(&strokes[ofst], tgt_time_sec);
            ofst += usb->getNumJoints();
        }
    }

    void sendVelocity(const std::vector<double>& command){
        if(!stroke_converter_){
            return;
        }
        stroke_converter_->calcActuatorVel(actuator_vel, command);

        //各USBで送信
        int ofst = 0;
        for(auto &usb:usbs){
            usb->sendVelocity(&actuator_vel[ofst]);
            ofst += usb->getNumJoints();
        }

        copy(joint_velocity,command);
    }

    void runScript(std::vector<uint16_t> script_no){
        int ofst = 0;
        for(auto &usb:usbs){
            usb->runScript(&script_no[ofst]);
            ofst += usb->getNumJoints();
        }
    }

    //非RT
    std::string getJointName(int idx){
        int ofst = 0;
        size_t usbidx = 0;
        for (usbidx = 0; usbidx < usbs.size(); ++usbidx) {
            auto usb = usbs[usbidx];
            if ((idx - ofst) >= usb->getNumJoints()) {
                ofst += usb->getNumJoints();
            } else {
                break;
            }
        }
        if (usbidx < usbs.size()) {
            auto name = usbs[usbidx]->getJointName(idx - ofst);
            return name;
        }

        return "";
    }

    //非RT
    std::vector<std::string> getJointNames(){
        std::vector<std::string> ret;
        for (auto &usb : usbs) {
            auto jnames = usb->getJointNames();
            ret.insert(ret.end(), jnames.begin(), jnames.end());
        }
        return ret;
    }


    std::string getPortName(int idx){
        return usbs[idx]->getPortName();
    }

    //cmdsの並び順は、各USBのMSの順番
    void getOtherCommands(std::vector<BuffList> &cmds) {
        size_t idx = 0;
        for (size_t usbidx = 0; usbidx < usbs.size(); ++usbidx) {
            size_t idx_min = idx;
            size_t idx_max = idx + usbs[usbidx]->getNumMs();
            for (; idx < idx_max && idx < cmds.size(); ++idx) {
                usbs[usbidx]->getOtherCommands(idx - idx_min, cmds[idx]);
            }
        }
    }

    //cmdsの並び順は、各USBのMSの順番
    void sendOtherCommands(std::vector<BuffList> &cmds) {
        size_t idx = 0;
        for (size_t usbidx = 0; usbidx < usbs.size(); ++usbidx) {
            size_t idx_min = idx;
            size_t idx_max = idx + usbs[usbidx]->getNumMs();
            for (; idx < idx_max && idx < cmds.size(); ++idx) {
                usbs[usbidx]->sendOtherCommands(idx - idx_min,cmds[idx]);
            }
        }
    }

    void send(){
        for(size_t idx = 0;idx < usbs.size();++idx){
            usbs[idx]->send();
        }
    }

private:
  pluginlib::ClassLoader<seed_converter::StrokeConverter> converter_loader_;
  boost::shared_ptr<seed_converter::StrokeConverter> stroke_converter_ = nullptr;

  std::vector<int16_t> actuator_vel;
  std::vector<double> joint_velocity;
  std::vector<double> joint_velocity_fb;

  std::vector<int16_t> strokes;
  std::vector<int16_t> positions_fb;
  std::vector<double> joint_position_fb;
  std::vector<double> joint_position_fb_prev1;
  std::vector<double> joint_position_fb_prev2;

  std::vector<RobotDriverSingleUSB*> usbs;
};
