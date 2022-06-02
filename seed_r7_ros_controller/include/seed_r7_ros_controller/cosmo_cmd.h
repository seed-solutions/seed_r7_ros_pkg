#ifndef _COSMO_CMD_H_
#define _COSMO_CMD_H_

#include <ros/ros.h>
#include "seed_r7_ros_controller/CosmoCommand.h"
#include "seed_r7_ros_controller/CosmoSelectNumber.h"

class CosmoCmd {
    enum {
        SUCCESS, FAIL, INVALID
    };

public:
    CosmoCmd(const ros::NodeHandle &_nh, robot_hardware::RobotHW *_in_hw) :
            nh_(_nh), hw_(_in_hw) {
        client = nh_.serviceClient<seed_r7_ros_controller::CosmoCommand>("/cosmo");

        cosmo_timer = nh_.createTimer(ros::Duration(1), &CosmoCmd::run, this);

        if (ros::ok()) {
            cosmo_srv = nh_.advertiseService("cosmo_candidate_selection", &CosmoCmd::doCommand, this);
        }
    }
    ~CosmoCmd() {

    }

    //CosmoCMDの送受信を定義、基本的には来たら返す
    void run(const ros::TimerEvent &_event) {

        while (1) {
            auto cmd = hw_->getCosmoCmd();
            if (cmd.header_type < 0 || cmd.cmd_str.empty()) {
                break;
            }
            exec_cmd(cmd);
        }
    }

    bool exec_cmd(CosmoCmdReqType cmd) {

        std::string tgt_cmd;
        std::string opt;
        std::string opt2;
        std::string cosmo_resp_str = "";

        //cosmoコマンドをパースして、上位アプリに要求
        if (parse_cmd_string(cmd.cmd_str, tgt_cmd, opt, opt2)) {
            seed_r7_ros_controller::CosmoCommand srv;
            srv.request.command = tgt_cmd;
            try {
                srv.request.no = std::stoi(opt);
            } catch (...) {
                srv.request.no = -1;
            }
            if (opt2 != "") {
                srv.request.name = opt2;
            }
            if (client.call(srv)) {
                if (srv.response.result.empty() || srv.response.result == "fail") { //resultに何も入っていないか、resultがinvalidだった場合
                    if (srv.response.reply == "" || srv.response.reply == "check_done")
                        cosmo_resp_str = "invalid"; //例え失敗でもreplyがある場合はそれを送信する
                    else
                        cosmo_resp_str = srv.response.reply;
                } else if (srv.response.result == "success") {
                    cosmo_resp_str = srv.response.reply;
                } else {
                    std::cout << "[ERROR:] Result Message is " << srv.response.result << ", this is unexpected message!" << std::endl;
                }
            }
        }

        //replyがある場合、実行結果をMSに送信
        if (cosmo_resp_str != "" && cosmo_resp_str != "invalid") {
            CosmoCmdRespType resp;
            std::string scen = "sena";
            if (scen.size() <= cmd.cmd_str.size() && std::equal(std::begin(scen), std::end(scen), std::begin(cmd.cmd_str))) {
                //senaから始まるコマンドの場合
                resp.header_type = 0;
                resp.addr = 16;
                resp.cmd_type = 1;
                resp.msid = cmd.msid;
                resp.cmd_str = cosmo_resp_str;
            } else {
                resp.header_type = 0;
                resp.addr = 16;
                resp.cmd_type = 1;
                resp.msid = cmd.msid;
                resp.cmd_str = cosmo_resp_str;
            }
            hw_->sendCosmoCmdResp(resp);
            ROS_INFO_STREAM("cosmo scen executed --> msid: "<<resp.msid<<"  cmd: "<<cmd.cmd_str<<" resp: "<<cosmo_resp_str);

            //restart必要コマンドを含んでいる場合はrestart要求をする
            std::vector<std::string> restartList{"task_done","scen_done","check_done"};
            for(auto tgt: restartList){
                int findPos = cosmo_resp_str.find(tgt);
                if (findPos != std::string::npos) {
                    //std::cout << "Restart要求は" << reqRestart() << ""<<std::endl;
                    while(!reqRestart()){
                        break;
                    }
                    break;
                }
            }

        } else if (cosmo_resp_str == "") {
            ROS_INFO_STREAM("NO REPLY, BUT SUCCESS");
        } else {
            ROS_INFO_STREAM("INVALID STATUS");
        }

        return true;

    }

    bool doCommand(seed_r7_ros_controller::CosmoSelectNumber::Request &req, seed_r7_ros_controller::CosmoSelectNumber::Response &res) {

        std::string prefix;
        if (req.command == "task") {
            prefix = "task_select_";
        } else if (req.command == "scene") {
            prefix = "scen_select_";
        }

        if (!prefix.empty()) {
            CosmoCmdRespType resp;
            resp.header_type = 1; //FEEF
            resp.addr = 16;
            resp.cmd_type = 1; //0xA1
            resp.msid = 0;
            resp.cmd_str = prefix + std::to_string(req.number) + "_" + req.name;
            hw_->sendCosmoCmdResp(resp);
            ROS_INFO_STREAM("cosmo resp : "<<resp.cmd_str);
        }

        res.result = "success";
        return true;
    }

private:
    bool parse_cmd_string(std::string str, std::string &tgt_cmd, std::string &opt, std::string &opt2) {

        tgt_cmd = "";
        opt = "";
        opt2 = "";
        std::vector < std::string > cmdList = { "robo_name_get", "task_play", "mtn_ply", "sena_start", "sena_pause", "sena_stop", "sena_restart", "scen_choice", "send_check", "chk_state" };

        for (auto cmd : cmdList) {
            //cmdの先頭がcmdListの候補の中と一致した場合
            if (cmd.size() <= str.size() && std::equal(std::begin(cmd), std::end(cmd), std::begin(str))) {
                tgt_cmd = cmd; //tgt_cmdにその一致した内容を入れる
                //cmdより長い場合分割して以降の文字を取得
                if (cmd.size() < str.size()) {
                    //アンダーバー_で分割
                    auto separator = std::string("_");         // 区切り文字
                    auto sep_length = separator.length(); // 区切り文字の長さ
                    auto seplist = std::vector<std::string>();
                    if (sep_length == 0) {
                        seplist.push_back(str.substr(cmd.size() + 1));
                    } else {
                        auto offset = std::string::size_type(0);
                        while (1) {
                            auto pos = str.substr(cmd.size() + 1).find(separator, offset);
                            if (pos == std::string::npos) {
                                seplist.push_back(str.substr(cmd.size() + 1).substr(offset));
                                break;
                            }
                            seplist.push_back(str.substr(cmd.size() + 1).substr(offset, pos - offset));
                            offset = pos + sep_length;
                        }
                    }
                    //_オプション(数字)がある場合
                    if (seplist.size() == 1) {
                        opt = seplist[0];
                    }

                    //_更にオプション（名前）がある場合
                    if (seplist.size() == 2) {
                        opt = seplist[0];
                        opt2 = seplist[1];
                    }
                }
                std::cout << "RECV SOME MSG: " << tgt_cmd << ", " << opt << ", " << opt2 << std::endl;
                return true;
            }
        }

        return false;
    }

    bool reqRestart(){
        seed_r7_ros_controller::CosmoCommand srv;
        srv.request.command = "sena_restart";
        srv.request.no = 0;
        if (client.call(srv)) {
            if (srv.response.result.empty() || srv.response.result == "fail") { //resultに何も入っていないか、resultがinvalidだった場合
                return false;
            } else if (srv.response.result == "success") {
                return true;
            } else {
                std::cout << "[ERROR:] Result Message is " << srv.response.result << ", this is unexpected message!" << std::endl;
                return false;
            }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient client;
    robot_hardware::RobotHW *hw_;
    ros::Timer cosmo_timer;
    ros::ServiceServer cosmo_srv;
};

#endif
