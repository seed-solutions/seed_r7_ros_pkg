#include <iostream>
#include <filesystem>

#include "motion_request_server.hpp"
#include "structure_otama_motion.hpp"
#include "fileio_json.hpp"
#include "aero_data.hpp"

namespace motion_player {

void dump(const char *str, uint8_t *data, int len) {
    printf("[%s] data size: %02d hex: ", str, len);
    for (int idx = 0; idx < len; ++idx) {
        if (idx % 4 == 0) {
            printf("  ");
        }
        printf("%02x", data[idx]);
    }
    printf("\n");
}

int sendArmTarget(double dt_sec, int step, double control_period_sec, std::vector<double> &prev_target, const std::vector<ArmTarget> &arm_tgts, const ArmInfoList &arm_info, const boost::shared_ptr<seed_converter::StrokeConverter> stroke_converter_, reqBuffType *req_buff) {
    auto jnames = arm_info.getJointNames();
    std::vector < int16_t > strokes;
    strokes.resize(jnames.size(), 0x7FFF);
    //アーム動作

    int send_num = 0;

    for (auto &arm_tgt : arm_tgts) {
        double duration_sec = dt_sec * (arm_tgt.step_end - step);
        uint16_t duration = static_cast<uint16_t>(duration_sec * 100.);

        for (size_t jidx = 0; jidx < jnames.size(); ++jidx) {
            auto jvalue = arm_tgt.tgt_state.get_position(jnames[jidx]);
            if (jvalue != JOINT_VALUE_INVALID) { // 動かさない軸はINVALIDが入っている可能性がある。その場合は、前回値を使う。
                prev_target[jidx] = jvalue;
            }
        }

        if (stroke_converter_) {
            stroke_converter_->Angle2Stroke(strokes, prev_target);
        }

        int idx = 0;
        for (size_t msidx = 0; msidx < arm_info.info.size(); ++msidx) {
            AeroDataReqType data;

            for (size_t jidx = 0; jidx < arm_info.info[msidx].jinfo.size(); ++jidx) {
                int sendid = arm_info.info[msidx].jinfo[jidx].sendid;
                data.setData(strokes[idx], sendid * 2);
                ++idx;
            }
            data.setData(duration, 60);
            data.addChecksum();
            Request req;
            req.msid = arm_info.info[msidx].msid;
            req.step = (step * dt_sec) / control_period_sec;
            memcpy(req.rawdata.data, data.serialize(), data.getTotalLen());
            req.rawdata.size = data.getTotalLen();

            bool ok = false;
            while (!ok) {
                ok = req_buff->writeFromA(req);
                usleep(1000);
            }
            ++send_num;
        }
    }

    return send_num;
}

int sendFingerTarget(double dt_sec, int step, double control_period_sec, const std::vector<boost::optional<FingerTarget>> &finger_tgts, FingerInfoList &finger_info, reqBuffType *req_buff) {
    size_t finger_id = 0;
    int send_num = 0;

    std::map<int,std::map<int,AeroDataReqType>> sendd_list;
    for (auto &finger_tgt_tmp : finger_tgts) {
        if (finger_tgt_tmp && finger_id < finger_info.info.size()) {
            FingerInfo &info = finger_info.info[finger_id];
            int msid = info.msid;
            auto& ms_sendd = sendd_list[msid];//msIDごとのデータを取得

            const FingerTarget &finger_tgt = finger_tgt_tmp.get();
            auto &data = ms_sendd[finger_tgt.step_end];//動作終了時刻ごとのデータを取得

            int tgtpos = info.min + (info.max - info.min) * finger_tgt.tgt_pos;
            if (tgtpos > std::numeric_limits < int16_t > ::max()) {
                tgtpos = std::numeric_limits < int16_t > ::max();
            } else if (tgtpos < std::numeric_limits < int16_t > ::min()) {
                tgtpos = std::numeric_limits < int16_t > ::min();
            }
            int16_t tgt_pos = static_cast<int16_t>(tgtpos);
            data.setData(tgt_pos, info.sendid * 2);

        }
        finger_id++;
    }


    for (auto &ms_sendd : sendd_list) {
        auto msid = ms_sendd.first;
        auto &ms_data = ms_sendd.second;
        for (auto &finger_data : ms_data) {
            auto step_end = finger_data.first;
            auto &data = finger_data.second;

            double duration_sec = dt_sec * (step_end - step);
            uint16_t duration = static_cast<uint16_t>(duration_sec * 100.);
            data.setData(duration, 60);
            data.addChecksum();

            //データをRT側に受け渡す
            Request req;
            req.msid = msid;
            req.step = (step * dt_sec) / control_period_sec;
            memcpy(req.rawdata.data, data.serialize(), data.getTotalLen());
            req.rawdata.size = data.getTotalLen();
            bool ok = false;
            while (!ok) {
                ok = req_buff->writeFromA(req);
                usleep(1000);
            }
            ++send_num;
        }
    }
    return send_num;
}

MotionBase* loadOtamaMotion(std::string &path_str, std::vector<std::string> joint_names, JointTarget &jtarget) {
    if(path_str.empty()){
        return nullptr;
    }
    OtamaMotion *otama_motion = new OtamaMotion;
    try {
        otama_motion->load(path_str);
        for (size_t idx = 0; idx < joint_names.size(); ++idx) {
            auto jvalue = otama_motion->default_state.get_position(joint_names[idx]);
            if (jvalue != JOINT_VALUE_INVALID) {
                jtarget.tgtpos[idx] = jvalue;
            } else {
                jtarget.tgtpos[idx] = 0;
            }
        }

    } catch (...) {
        delete otama_motion;
        otama_motion = nullptr;
    }
    return otama_motion;
}


MotionBase* loadKeyframeMotion(std::string &path_str, std::vector<std::string> joint_names, JointTarget &jtarget) {
    if (path_str.empty()) {
        return nullptr;
    }

    KeyframeMotion *keyframe_motion = new KeyframeMotion;

    try {
        keyframe_motion->load(path_str);

        if (keyframe_motion->points.size() != 0) {

            for (size_t idx = 0; idx < joint_names.size(); ++idx) {
                jtarget.tgtpos[idx] = 0;
            }
        }else{
            delete keyframe_motion;
            keyframe_motion = nullptr;
        }

    } catch (...) {
        delete keyframe_motion;
        keyframe_motion = nullptr;
    }

    return keyframe_motion;
}


FingerInfoList createFingerInfo(ros::NodeHandle &my_nh) {
    FingerInfoList finger_info;
    FingerInfo finger1_info { .msid = 0, .sendid = 12, .min = 800, .max = 3000 };
    FingerInfo finger2_info { .msid = 0, .sendid = 13, .min = 800, .max = 3000 };
    FingerInfo finger3_info { .msid = 0, .sendid = 14, .min = 1000, .max = 4500 };
    FingerInfo finger4_info { .msid = 0, .sendid = 28, .min = -6000, .max = -5000 };
    FingerInfo finger5_info { .msid = 0, .sendid = 29, .min = 2000, .max = 1500 };

    finger_info.info.push_back(finger1_info);
    finger_info.info.push_back(finger2_info);
    finger_info.info.push_back(finger3_info);
    finger_info.info.push_back(finger4_info);
    finger_info.info.push_back(finger5_info);

    return finger_info;

}

ArmInfoList createArmInfo(ros::NodeHandle &my_nh) {
    ArmInfoList ret;
    XmlRpc::XmlRpcValue ms_settings;
    my_nh.getParam("/ms_settings", ms_settings);
    for (auto itr = ms_settings.begin(); itr != ms_settings.end(); ++itr) {
        ArmInfo arminfo;

        auto key = itr->first;
        XmlRpc::XmlRpcValue value = itr->second;
        int msid = int(value["msid"]);
        arminfo.msid = msid;

        XmlRpc::XmlRpcValue joints = value["joints"];
        for (int jidx = 0; jidx < joints.size(); ++jidx) {
            XmlRpc::XmlRpcValue joint = joints[jidx];
            std::string jname = std::string(joint["name"]);
            int aero_idx = int(joint["aero_idx"]);

            JointInfo jinfo;
            jinfo.name = jname;
            jinfo.sendid = aero_idx;
            arminfo.jinfo.push_back(jinfo);
        }

        ret.info.push_back(arminfo);
    }
    return ret;
}

MotionRequestServer::~MotionRequestServer() {
    shutdown.store(true);
    mtx.lock();
    cond.notify_one();
    mtx.unlock();
    if (send_thread.joinable()) {
        send_thread.join();
    }

    mtx.lock();
    if (cur_motion) {
        cur_miter = nullptr;
        delete cur_motion;
        cur_motion = nullptr;
    }
    mtx.unlock();

    delete extension_list;
}

bool MotionRequestServer::load(motion_player::Load::Request &req, motion_player::Load::Response &res) {
    std::string file_path = req.file_path;

    ROS_INFO_STREAM("motion file is to be loaded :"<<file_path);


    std::lock_guard<std::mutex> guard(mtx);
    if (cur_motion) {
        cur_miter = nullptr;
        delete cur_motion;
        cur_motion = nullptr;
    }
    std::filesystem::path fpath = file_path;
    if(fpath.extension() == ".kf"){
        auto path_str = fpath.replace_extension(".json").string();
        cur_motion = loadKeyframeMotion(path_str, joint_names, jtarget);
    }else if(fpath.extension() == ".otm"){
        auto path_str = fpath.replace_extension(".json").string();
        cur_motion = loadOtamaMotion(path_str, joint_names, jtarget);

        //デバイス設定のロード
        auto dev_setting_path = fpath.parent_path().string() + "/device.settings";
        std::ifstream dev_setting_rfile(dev_setting_path, std::ios::in);
        if (dev_setting_rfile) {
            std::string dev_setting_str((std::istreambuf_iterator<char>(dev_setting_rfile)), std::istreambuf_iterator<char>());
            JsonInputArchive ar(dev_setting_str);
            ar >> finger_info;
        }
    }



    if (cur_motion) {
        cur_miter = cur_motion->getIterator();
        dt_sec = cur_miter->getStepTime();
        res.result = true;
    }else{
        ROS_ERROR_STREAM("failed to load motion : "<<file_path);
        res.result = false;
    }

    return true;
}

void MotionRequestServer::execute(const motion_player::PlayGoalConstPtr &goal) {
    motion_player::PlayResult result;
    motion_player::PlayFeedback feedback;

    if (!cur_motion || !cur_miter) {
        play_as->setAborted(result, "Motion has not loaded yet.");
        return;
    }

    mtx.lock();
    start_step = goal->start_step;
    int step_max = cur_miter->getMaxStep();
    cond.notify_one();

    EXTENSION_PARAM_LIST empty;
    std::swap(extension_params, empty);

    mtx.unlock();

    int cycle_noresp = 0;

    extension_list->entry();

    int extension_cur_step = -1;
    EXT_PARAMS extension_cur_param;

    while (1) {

        if (!cancel.load() && play_as->isPreemptRequested()) {
            cancel.store(true);

            stop_request->store(true); //RT側に終了要求を出す
            req_buff->setPermissionRead(false); //読み出し不可にしておく
            req_buff->clear(); //書き込み側が処理完了できるように、全データを削除しておく
            mtx.lock(); //書き込み完了待ち
            req_buff->clear(); //書き込まれたデータをすぐ消去する
            mtx.unlock();
        }

        if(cancel.load() && cycle_noresp > 1000){
            //モーションが再生される前の停止要求がありうるので、少し待って応答が帰ってこない場合はループを抜ける
            resp_buff->clear();
            break;
        }

        auto resp = resp_buff->readFromB(false);
        if (!resp) {
            cycle_noresp++;
            usleep(1000);
            continue;
        }
        cycle_noresp = 0;

        feedback.step = resp->step * control_period_sec / dt_sec;
        if (step_max > 0) {
            feedback.percent = static_cast<uint8_t>((100 * feedback.step) / step_max);
        }

        while (extension_cur_step <= static_cast<int>(feedback.step)) {
            //実行ステップであれば、実行する
            if (0 <= extension_cur_step) {
                extension_list->execute(extension_cur_step, extension_cur_param);
            }

            //次のものが読み込み可能であれば、読み込む
            extension_mtx.lock();
            if (!extension_params.empty()) {
                auto next_extension = extension_params.front();
                extension_cur_param = next_extension.*EXTENSION_PARAMS;
                extension_cur_step = next_extension.*EXTENSION_STEP;
                extension_params.pop();
            }else{
                extension_cur_step = -1;
            }
            extension_mtx.unlock();

            if(extension_cur_step < 0){
                break;
            }
        }


        play_as->publishFeedback(feedback);

        if (resp->done) {
            break;
        }
    }

    extension_list->exit();

    if (!cancel.load()) {
        play_as->setSucceeded(result);
        ROS_INFO("motion was successfully completed");
    } else {
        play_as->setPreempted();
        cancel.store(false);
        ROS_INFO("motion was canceled");
    }

}

void MotionRequestServer::sendThread() {
    while (1) {
        std::unique_lock < std::mutex > lk(mtx);
        cond.wait(lk);
        if (shutdown.load()) {
            break;
        }

        if (!cur_motion || !cur_miter || cancel.load()) {
            continue;
        }


        req_buff->setPermissionRead(false);
        req_buff->clear();

        int send_size = 0;
        bool readable = false;
        double dt_sec = cur_miter->getStepTime();
        int step_max = cur_miter->getMaxStep();
        for (int step = start_step; step <= step_max; ++step) {
            if (cancel.load() || shutdown.load()) {
                break;
            }

            if (cur_miter->hasElement(step)) {
                //ファイル読み込みを待つ
                while(!cur_miter->valueAvailable()){
                    usleep(1000);
                }

                TargetData data = cur_miter->getValue();

                //ここでバッファの書き込み待ちが発生する
                send_size += sendArmTarget(dt_sec, step, control_period_sec, jtarget.tgtpos, data.arm_tgt, arm_info, stroke_converter_, req_buff);
                send_size += sendFingerTarget(dt_sec, step, control_period_sec, data.finger_tgt, finger_info, req_buff);

                extension_mtx.lock();
                if (!data.extension.empty()) {
                    extension_params.push(std::make_pair(step, data.extension));
                }
                extension_mtx.unlock();

                if ((send_size >= REQ_BUFF_SIZE/2 || step >= step_max) && !readable) {
                    req_buff->setPermissionRead(true);//データが十分に溜まったら、読み出し可能にする
                    readable = true;
                }

            }
            //最後に空リクエストを入れておく
            if (step == step_max) {
                Request req;
                req.msid = -1;
                req.step = step_max * dt_sec / control_period_sec;
                req_buff->writeFromA(req);
            }
        }
    }

}

void MotionRequestServer::init(ros::NodeHandle &my_nh, reqBuffType *req_buff, respBuffType *resp_buff,std::atomic<bool>* stop_request) {
    this->req_buff = req_buff;
    this->resp_buff = resp_buff;
    this->stop_request = stop_request;

    ros::NodeHandle seed_nh("/seed_ros_controller");

    finger_info = createFingerInfo(my_nh);
    arm_info = createArmInfo(my_nh);
    this->joint_names = arm_info.getJointNames();
    jtarget.tgtpos.resize(joint_names.size(), 0);

    seed_nh.param("control_period", control_period_sec, control_period_sec);

    std::string robot_model_plugin_name;
    if (seed_nh.getParam("robot_model_plugin", robot_model_plugin_name)) {
        try {
            stroke_converter_ = converter_loader_.createInstance(robot_model_plugin_name);
            stroke_converter_->initialize(seed_nh);
            stroke_converter_->setJointNames(joint_names);
        } catch (pluginlib::PluginlibException &ex) {
            ROS_ERROR("The plugin failed to load : %s", ex.what());
        }
    } else {
        ROS_ERROR_STREAM("robot_model_plugin param is not set");
    }

    load_srv = my_nh.advertiseService("load", &MotionRequestServer::load, this);
    play_as = PlayActionServerPtr(new PlayActionServer(my_nh, "play", boost::bind(&MotionRequestServer::execute, this, _1), false));
    play_as->start();

    send_thread = std::thread(&MotionRequestServer::sendThread, this);

}

}
