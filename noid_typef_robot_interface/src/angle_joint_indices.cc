/* auto generating */

#include "angle_joint_indices.hh"

using namespace noid;
using namespace controller;

SEEDProtocol::SEEDProtocol(const std::string& _port, uint _id)
{
}

SEEDProtocol::~SEEDProtocol()
{
}



NoidUpperController::NoidUpperController(const std::string& _port) : SEEDProtocol(_port, ID_UPPER)
{
   
   //depends on robot model
   angle_joint_indices_["waist_y_joint"] = 0;
   angle_joint_indices_["waist_p_joint"] = 1;
   angle_joint_indices_["waist_r_joint"] = 2;
   angle_joint_indices_["l_shoulder_p_joint"] = 3;
   angle_joint_indices_["l_shoulder_r_joint"] = 4;
   angle_joint_indices_["l_shoulder_y_joint"] = 5;
   angle_joint_indices_["l_elbow_joint"] = 6;
   angle_joint_indices_["l_wrist_y_joint"] = 7;
   angle_joint_indices_["l_wrist_p_joint"] = 8;
   angle_joint_indices_["l_wrist_r_joint"] = 9;
   angle_joint_indices_["l_indexbase_joint"] = 10;
   angle_joint_indices_["l_indexmid_joint"] = 11;
   angle_joint_indices_["l_indexend_joint"] = 12;
   angle_joint_indices_["l_thumb_joint"] = 13;
   angle_joint_indices_["neck_y_joint"] = 14;
   angle_joint_indices_["neck_p_joint"] = 15;
   angle_joint_indices_["neck_r_joint"] = 16;
   angle_joint_indices_["r_shoulder_p_joint"] = 17;
   angle_joint_indices_["r_shoulder_r_joint"] = 18;
   angle_joint_indices_["r_shoulder_y_joint"] = 19;
   angle_joint_indices_["r_elbow_joint"] = 20;
   angle_joint_indices_["r_wrist_y_joint"] = 21;
   angle_joint_indices_["r_wrist_p_joint"] = 22;
   angle_joint_indices_["r_wrist_r_joint"] = 23;
   angle_joint_indices_["r_indexbase_joint"] = 24;
   angle_joint_indices_["r_indexmid_joint"] = 25;
   angle_joint_indices_["r_indexend_joint"] = 26;
   angle_joint_indices_["r_thumb_joint"] = 27;
}

NoidUpperController::~NoidUpperController()
{

}

NoidLowerController::NoidLowerController(const std::string& _port) : 
    SEEDprotocol(_port, ID_LOWER)
{
     angle_joint_indices_["knee_joint"] = 28;
     angle_joint_indices_["ankle_joint"] = 29;
}

NoidLowerController::~NoidLowerController()
{
}





