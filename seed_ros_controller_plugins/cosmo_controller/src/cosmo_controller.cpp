#include <pluginlib/class_list_macros.hpp>
#include "cosmo_controller.hpp"


namespace cosmo_controller {

CosmoController::CosmoController(){
};

bool CosmoController::init(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){
    client = root_nh.serviceClient<cosmo_controller::CosmoCommand>("cosmo");
    cosmo_srv = root_nh.advertiseService("cosmo_candidate_selection", &CosmoController::doCommand, this);
    return true;
}


void CosmoController::execute(const BuffRaw* buf_recv,BuffRaw* buf_send){
    CosmoCmdReqType  cosmo_req;
    CosmoCmdRespType cosmo_resp;

    cosmo_req.fromRawBuffer(*buf_recv);
    auto has_resp = exec_cmd(cosmo_req,cosmo_resp);

    if(has_resp){
        cosmo_resp.toRawBuffer(*buf_send);
    }
}

}

PLUGINLIB_EXPORT_CLASS(cosmo_controller::CosmoController, controller_interface::ControllerBase)
