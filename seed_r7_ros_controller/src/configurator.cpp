#include "seed_r7_ros_controller/configurator.h"


robot_hardware::Configurator::Configurator(const ros::NodeHandle &_nh, robot_hardware::RobotHW *_in_hw):nh_(_nh) ,hw_(_in_hw){
    srv_sysconfig_write = nh_.advertiseService("ms_sysconfig_write", &Configurator::ConfiguratorSysconfigWriteCallback,this);
    srv_sysconfig_read = nh_.advertiseService("ms_sysconfig_read", &Configurator::ConfiguratorSysconfigReadCallback,this);
    srv_config_write = nh_.advertiseService("ms_config_write", &Configurator::ConfiguratorConfigWriteCallback,this);
    srv_config_read = nh_.advertiseService("ms_config_read", &Configurator::ConfiguratorConfigReadCallback,this);

}

robot_hardware::Configurator::~Configurator() {
}

bool robot_hardware::Configurator::ConfiguratorSysconfigWriteCallback(seed_r7_ros_controller::MsSystemConfigWrite::Request &_req, seed_r7_ros_controller::MsSystemConfigWrite::Response &_res) {
    uint8_t ms_bit = 0;
    for(auto ms:_req.config.ms){
        ms_bit |= (0x01 << (ms.msid -1));
    }

    for(auto ms:_req.config.ms){
        uint8_t own_bit = 0x01 << (ms.msid -1);
        uint8_t sms_bit = ms_bit ^ own_bit;

        uint8_t data_1byte[32] = {0};
        data_1byte[0] = ms.msid; //msid
        data_1byte[1] = sms_bit; //smsid
        data_1byte[2] = ms.type;//機種
        data_1byte[3] = _req.config.protocol;//AEROプロトコル
        data_1byte[4] = _req.config.use_cloud;//cloudの利用

        uint16_t address = 0x0001;//システム設定

        hw_->stopCyclic(true);
        if(ms.port == "/dev/upper_body"){
            hw_->write_1byte_upper(address,data_1byte,5);
            hw_->resetting_upper();
        }else if(ms.port == "/dev/lower_body"){
            hw_->write_1byte_lower(address,data_1byte,5);
            hw_->resetting_lower();
        }else{
            ROS_INFO_STREAM("port:"<<ms.port<<"has not be supported yet.");
        }
        usleep(1000000);//MSリセット完了まで少し待つ
        hw_->stopCyclic(false);
    }

    return true;
}


bool robot_hardware::Configurator::ConfiguratorSysconfigReadCallback(seed_r7_ros_controller::MsSystemConfigRead::Request& _req, seed_r7_ros_controller::MsSystemConfigRead::Response& _res){
    static constexpr uint16_t address = 0x0000;//システム設定

    hw_->stopCyclic(true);
    std::vector<uint8_t> data_upper = hw_->read_1byte_upper(address,32);
    if (data_upper.size() != 0) {
        seed_r7_ros_controller::MsSystemConfig msconfig;
        msconfig.port = "/dev/upper_body";
        msconfig.msid = data_upper[1];
        msconfig.type = data_upper[3];
        _res.config.ms.push_back(msconfig);

        if (_res.config.protocol == 0) {
            _res.config.protocol = data_upper[4];
        }

        if (_res.config.use_cloud == 0) {
            _res.config.use_cloud = data_upper[5];
        }
    }

    std::vector<uint8_t> data_lower = hw_->read_1byte_lower(address,32);
    if (data_lower.size() != 0) {
        seed_r7_ros_controller::MsSystemConfig msconfig;
        msconfig.port = "/dev/lower_body";
        msconfig.msid = data_lower[1];
        msconfig.type = data_lower[3];
        _res.config.ms.push_back(msconfig);

        if (_res.config.protocol == 0) {
            _res.config.protocol = data_lower[4];
        }

        if (_res.config.use_cloud == 0) {
            _res.config.use_cloud = data_lower[5];
        }
    }
    hw_->stopCyclic(false);

    return true;
}

bool robot_hardware::Configurator::ConfiguratorConfigWriteCallback(seed_r7_ros_controller::MsConfigWrite::Request& _req, seed_r7_ros_controller::MsConfigWrite::Response& _res){

    uint8_t data_1byte[64] = {0};
    int size = _req.data.size();
    if(size > 64){
        return false;
    }
    memcpy(data_1byte,_req.data.data(),_req.data.size());
    uint16_t address = _req.address;

    hw_->stopCyclic(true);
    if(_req.port == "/dev/upper_body"){
        hw_->write_1byte_upper(address,data_1byte,_req.data.size());
    }else if(_req.port == "/dev/lower_body"){
        hw_->write_1byte_lower(address,data_1byte,_req.data.size());
    }else{
        ROS_INFO_STREAM("port:"<<_req.port<<"has not be supported yet.");
    }
    usleep(1000);//MSセット完了まで少し待つ
    hw_->stopCyclic(false);
    return true;
}

bool robot_hardware::Configurator::ConfiguratorConfigReadCallback(seed_r7_ros_controller::MsConfigRead::Request& _req, seed_r7_ros_controller::MsConfigRead::Response& _res){
    uint16_t address = _req.address;
    uint16_t size = _req.size;

    hw_->stopCyclic(true);
    if (_req.port == "/dev/upper_body") {
        _res.data = hw_->read_1byte_upper(address,size);
    } else if (_req.port == "/dev/lower_body") {
        _res.data = hw_->read_1byte_lower(address,size);
    }else{
        ROS_INFO_STREAM("port:"<<_req.port<<"has not be supported yet.");
    }

    hw_->stopCyclic(false);

    return true;
}


