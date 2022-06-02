#pragma once

#include "aero_conversion.hpp"
#include "aero3_command.hpp"

class MSDriver {
public:
    MSDriver(XmlRpc::XmlRpcValue &settings) {

        if (!settings["msid"].valid()) {
            return;
        }
        msid = int(settings["msid"]);

        std::vector<unsigned int> aero_index;
        if (settings["joints"].valid()) {
            auto joints_params = settings["joints"];
            for (int idx = 0; idx < joints_params.size(); ++idx) {
                auto joint_params = joints_params[idx];
                if(!joint_params["name"].valid() || !joint_params["aero_idx"].valid()){
                    continue;
                }

                auto name = std::string(joint_params["name"]);
                auto aero_idx = static_cast<unsigned int>(int(joint_params["aero_idx"]));

                joint_names.push_back(name);
                aero_index.push_back(aero_idx);

            }
        }

        aero_conversion = new AeroConversion(aero_index);

        aero_strokes.resize(31,0x7FFF);
        prev_strokes.resize(31,0x7FFF);

        aero_velocities.resize(31,0x7FFF);
        prev_velocities.resize(31,0x7FFF);

        current_positions.resize(31,0x0000);
        current_velocities.resize(31,0x0000);

        aero_scripts.resize(31,0x7FFF);
    }

    int getMsId(){
         return msid;
     }

    void getPosition(int16_t* ros_pos,AeroCommand *aero_driver){
        for(int idx = 0;idx < 31;++idx){
            auto pos = aero_driver->getpos(msid,idx);
            if(pos != 0x7FFF){
                current_positions[idx] = pos;
            }
        }
        aero_conversion->remapAeroToRos(ros_pos,current_positions);
    }

    void getMsStatus(uint16_t &status, AeroCommand *aero_driver) {
        status = aero_driver->getstatus(msid);
    }

//速度は取得できない
//    void getActualVelocities(std::vector<int16_t> &ros_velocities,AeroCommand *aero_driver){
//        for(int idx = 0;idx < 31;++idx){
//            if(aero_velocities[idx] != 0x7FFF){//移動に利用したものを、現在値として利用
//                current_velocities[idx] = aero_velocities[idx];
//            }
//        }
//
//        aero_conversion->remapAeroToRos(ros_velocities,current_velocities);
//    }

    int getNumJoints(){
        return joint_names.size();
    }

    //非RT
    std::string getJointName(int idx){
        size_t idx_tmp = static_cast<size_t>(idx);
        if(0<= idx_tmp && idx_tmp < joint_names.size()){
            return joint_names[idx_tmp];
        }
        return "";
    }

    //非RT
    std::vector<std::string> getJointNames(){
        return joint_names;
    }

    void sendPosition(const int16_t* ros_strokes, const double &tgt_time_sec,AeroCommand *aero_driver) {

        aero_conversion->remapRosToAero(aero_strokes,ros_strokes);
        size_t idxMax = aero_strokes.size() < prev_strokes.size() ? aero_strokes.size(): prev_strokes.size();

        bool exist_sendd = false;
        //前回送信値と同じであれば、送信しない。
        for (size_t idx = 0; idx < idxMax; ++idx) {
            if (aero_strokes[idx] == prev_strokes[idx]) {
                aero_strokes[idx] = 0x7FFF; // 値が同じであれば、送信しない。
            } else {
                prev_strokes[idx] = aero_strokes[idx];
                exist_sendd = true;
            }
        }

        //ロボットに送信
        if (exist_sendd) {
            has_sendd = true;
            aero_driver->sendMOVE(msid, tgt_time_sec, aero_strokes.data());
        }
    }

    void sendVelocity(const int16_t* ros_velocities,AeroCommand *aero_driver){
        aero_conversion->remapRosToAero(aero_velocities,ros_velocities);
        size_t idxMax = aero_velocities.size() < prev_velocities.size() ? aero_velocities.size(): prev_velocities.size();
        bool exist_sendd = false;
        //前回送信値と同じであれば、送信しない。
        for (size_t idx = 0; idx < idxMax; ++idx) {
            if (aero_velocities[idx] == prev_velocities[idx] && (aero_velocities[idx] == 0x0000 ||  aero_velocities[idx] == 0x7FFF)) {
                aero_velocities[idx] = 0x7FFF; // 値が同じであれば、送信しない。
            } else {
                prev_velocities[idx] = aero_velocities[idx];
                exist_sendd = true;
            }
        }

        //ロボットに送信
        if (exist_sendd) {
            has_sendd = true;
            aero_driver->sendTURN(msid, aero_velocities.data());
        }
    }

    void runScript(const uint16_t* ros_scripts,AeroCommand *aero_driver){
        aero_conversion->remapRosToAero(aero_scripts,ros_scripts);
        size_t idxMax = aero_scripts.size();

        //送信データが存在するかを確認
        bool exist_sendd = false;
        for (size_t idx = 0; idx < idxMax; ++idx) {
            if (aero_scripts[idx] != 0x7FFF) {
                exist_sendd = true;
                break;
            }
        }

        //ロボットに送信
        if(exist_sendd){
            has_sendd = true;
            aero_driver->sendSCRIPT(msid, 0, aero_scripts.data());
        }
    }


    bool getOtherCommands(BuffList &cmds, AeroCommand *aero_driver) {
        aero_driver->getOtherCommands(msid, cmds);
        return true;
    }


    bool sendOtherCommands(BuffList &cmds, AeroCommand *aero_driver) {
        if (cmds.size != 0) {
            has_sendd = true;
            aero_driver->sendOtherCommands(msid, cmds);
        }
        return true;
    }

    void send(AeroCommand *aero_driver){
        //送信データがなければ、状態取得コマンドを発行する
        if (!has_sendd) {
            aero_driver->sendPGET(msid);
        }
        has_sendd = false;
    }

private:
    bool has_sendd = false;

    int msid;
    AeroConversion *aero_conversion = nullptr;


    std::vector < std::string > joint_names;

    std::vector<uint16_t> aero_scripts;

    std::vector<int16_t> aero_velocities;
    std::vector<int16_t> prev_velocities;

    std::vector<int16_t> aero_strokes;
    std::vector<int16_t> prev_strokes;

    std::vector<int16_t> current_positions;
    std::vector<int16_t> current_velocities;

};
