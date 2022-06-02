#pragma once

#include <hardware_interface/other_command_interface_helper.hpp>

#include "cosmo_controller/CosmoCommand.h"
#include "cosmo_controller/CosmoSelectNumber.h"
#include "cosmo_data.hpp"

namespace cosmo_controller{

  class CosmoController
      : public controller_interface::OtherCommandInterfaceHelper
  {
  public:
    CosmoController();

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void execute(const BuffRaw* buf_recv,BuffRaw* buf_send) override;

    std::vector<std::pair<uint8_t,uint8_t>> handleHeaders() override{
        return {{0xef,0xfe},{0xfe,0xef}};
    }

  private:
    bool doCommand(cosmo_controller::CosmoSelectNumber::Request &req, cosmo_controller::CosmoSelectNumber::Response &res) {
        std::string prefix;
        if (req.command == "task") {
            prefix = "task_select_";
        } else if (req.command == "scene") {
            prefix = "scen_select_";
        }

        if (!prefix.empty()) {
            CosmoCmdRespType resp;
            resp.header_type = 1; //FEEF(なぜか、これだけはFEEFで送信)
            resp.addr = 16;
            resp.cmd_type = 0xa1; //0xA1
            resp.msid = 0;
            resp.cmd_str = prefix + std::to_string(req.number) + "_" + req.name;
            ROS_INFO_STREAM("cosmo resp : "<<resp.cmd_str);


            BuffRaw buff;
            resp.toRawBuffer(buff);
            addSendData(0, buff);//msid:0 = mmsに送信する
            res.result = "success";
        }else{
            res.result = "fail(no such command)";
        }


        return true;
    }


    //strをdelimで分割し、各要素を応答する
    std::vector < std::string > split(std::string &str, const std::string &delim) {
        std::vector < std::string > result;
        //cmdより長い場合分割して以降の文字を取得
        //アンダーバー_で分割
        auto delim_length = delim.length(); // 区切り文字の長さ
        if (delim_length == 0) {
            return result;
        } else {
            auto prev_pos = std::string::size_type(0);
            while (1) {
                auto cur_pos = str.find(delim, prev_pos);
                if (cur_pos == std::string::npos) {
                    result.push_back(str.substr(prev_pos));//prev_pos以降を格納
                    break;
                }
                result.push_back(str.substr(prev_pos, cur_pos - prev_pos));//prev_posからcur_pos-1までを格納
                prev_pos = cur_pos + delim_length;
            }
        }
        return result;
    }


    bool parse_cmd_string(std::string str, std::string &tgt_cmd, std::string &opt1, std::string &opt2) {

        tgt_cmd = "";
        opt1 = "";
        opt2 = "";
        std::vector < std::string > cmdList = { "robo_name_get", "task_play", "mtn_ply", "sena_start", "sena_pause", "sena_stop", "sena_restart", "scen_choice", "send_check", "chk_state" };

        for (auto &cmd : cmdList) {
            //cmdの先頭がcmdListの候補の中と一致した場合
            if (cmd.size() <= str.size() && std::equal(std::begin(cmd), std::end(cmd), std::begin(str))) {
                tgt_cmd = cmd; //tgt_cmdにその一致した内容を入れる
                if (cmd.size() < str.size()) {
                    auto substr = str.substr(cmd.size() + 1);
                    auto args = split(substr, "_");
                    if (args.size() >= 1) {
                        opt1 = args[0];
                    }
                    if (args.size() >= 2) {
                        for(int i = 1; i < args.size(); i++){
                            opt2 += args[i];
                            if(i != args.size() - 1){
                                opt2 += "_";
                            }
                        }
                    }
                }

                ROS_INFO_STREAM("received cosmo message : "<<tgt_cmd << ", " << opt1 << ", " << opt2);
                return true;
            }
        }

        return false;
    }

    bool reqRestart(){

        while (1) {
            cosmo_controller::CosmoCommand srv;
            srv.request.command = "sena_restart";
            srv.request.no = 0;
            if (client.call(srv)) {
                if (srv.response.result == "success") {
                    return true;
                } else if (srv.response.result.empty() || srv.response.result == "fail") { //resultに何も入っていないか、resultがinvalidだった場合
                    continue;
                } else {
                    ROS_ERROR_STREAM("reply message is unexpected.");
                    return false;
                }
            } else {
                ROS_ERROR_STREAM("service can not be called.");
                return false;
            }
        }

        return false;
    }


    std::string request_cmd(CosmoCmdReqType &cmd){
        std::string tgt_cmd;
        std::string opt1;
        std::string opt2;
        std::string cosmo_resp_str = "invalid";

        //cosmoコマンドをパースして、上位アプリに要求
        if (parse_cmd_string(cmd.cmd_str, tgt_cmd, opt1, opt2)) {
            cosmo_controller::CosmoCommand srv;
            srv.request.command = tgt_cmd;
            try {
                srv.request.no = std::stoi(opt1);
            } catch (...) {
                srv.request.no = -1;
            }
            srv.request.name = opt2;

            if (client.call(srv)) {
                if (srv.response.result == "success" || (!srv.response.result.empty() && srv.response.result != "fail" && !srv.response.reply.empty() && srv.response.reply != "check_done")) {
                    cosmo_resp_str = srv.response.reply;
                }
            }else{
                ROS_ERROR_STREAM("service can not be called.");
            }
        }


        bool need_restart = false;
        //restart必要コマンドを含んでいる場合はrestart要求をする
        std::vector < std::string > restartList { "task_done", "scen_done", "check_done" };
        for (auto tgt : restartList) {
            std::string::size_type findPos = cosmo_resp_str.find(tgt);
            if (findPos != std::string::npos) {
                need_restart = true;
                break;
            }
        }

        if (need_restart) {
            reqRestart();
        }

        return cosmo_resp_str;
    }

    bool reply_cmd(std::string cosmo_resp_str,CosmoCmdRespType &resp){
        bool need_reply = false;
        if (cosmo_resp_str.empty()) {
            ROS_INFO_STREAM("The cosmo execution succeeded. There is no cosmo reply message.");
        } else if (cosmo_resp_str == "invalid") {
            ROS_ERROR_STREAM("The cosmo request was executed, but the reply message was invalid.");
        } else {
            //replyがある場合、実行結果をMSに送信
            resp.cmd_str = cosmo_resp_str;
            need_reply = true;
        }

        return need_reply;
    }

    bool exec_cmd(CosmoCmdReqType &cmd,CosmoCmdRespType &resp) {
        bool need_reply = false;
        resp.header_type = 0;
        resp.addr = 16;
        resp.cmd_type = 0xa1;
        resp.msid = cmd.msid;

        ROS_INFO_STREAM("cosmo command : "<<cmd.cmd_str);

        std::string cosmo_resp_str = request_cmd(cmd);
        need_reply = reply_cmd(cosmo_resp_str, resp);
        if (need_reply) {
            ROS_INFO_STREAM("cosmo scen executed --> msid: "<<resp.msid<<"  cmd: "<<cmd.cmd_str<<" resp: "<<cosmo_resp_str);
        }

        return need_reply;
    }

  private:
    ros::ServiceClient client;
    ros::ServiceServer cosmo_srv;
  };

}
