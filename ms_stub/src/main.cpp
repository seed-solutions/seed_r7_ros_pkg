
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpc.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <filesystem>

#include "ms_stub.hpp"
#include "cosmo_joy.hpp"
#include "cosmo.hpp"



int main(int argc, char **argv){
    ros::init(argc, argv, "ms_stub");

    ros::NodeHandle root_nh("/");
    ros::NodeHandle my_nh("~");

    XmlRpc::XmlRpcValue usb_settings;
    XmlRpc::XmlRpcValue ms_settings;

    root_nh.getParam("usb_settings",usb_settings);
    root_nh.getParam("ms_settings",ms_settings);

    Cosmo cosmo;
    MSStubSingle *joy_usb = nullptr;

    for (int usb_idx = 0; usb_idx < usb_settings.size(); ++usb_idx) {
        auto param = usb_settings[usb_idx];
        if (!param["port"].valid() || !param["mslist"].valid()) {
            continue;
        }

        auto usb_port = std::string(param["port"]);
        if (usb_port[0] == '/') {
            usb_port = "./tmp" + usb_port;
        }

        std::filesystem::path path = usb_port;
        std::filesystem::path abspath = std::filesystem::absolute(usb_port);

        usb_settings[usb_idx]["port"] = abspath.string();
        std::filesystem::create_directories(path.parent_path());


        MSStubSingle *usbDriver = new MSStubSingle(usb_port);
        if(!joy_usb)joy_usb = usbDriver;


        XmlRpc::XmlRpcValue mslist = param["mslist"];
        for (int ms_idx = 0; ms_idx < mslist.size(); ++ms_idx) {
            auto key = std::string(mslist[ms_idx]);
            if (ms_settings[key].valid()) {
                auto ms_setting = ms_settings[key];
                int msid = int(ms_setting["msid"]);
                usbDriver->addMsId(msid);
            }
        }

        cosmo.addUsb(usbDriver);

    }
    root_nh.setParam("usb_settings", usb_settings);

    ros::Duration(5).sleep(); //他のプロセスが立ち上がってから、cosmoの処理を始める

    CosmoJoy joy(my_nh,joy_usb);
    cosmo.run();

    ros::spin();
}
