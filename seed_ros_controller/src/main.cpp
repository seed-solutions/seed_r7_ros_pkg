#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <QApplication>

#include "timer_lib.hpp"
#include "hardware_adaptor.hpp"

std::atomic<bool> is_shutdown = false;

void sig_handler(int signo){
    is_shutdown.store(true);
    ros::shutdown();
}

void rt_loop(hardware_interface::RobotHW &hw,controller_manager::ControllerManager &cm,double period_sec){
    ROS_INFO("ControllerManager start with %f Hz", 1.0 / period_sec);

    long period_usec = period_sec * USEC_PER_SEC;
    long period_ns = period_sec * NSEC_PER_SEC;
    timespec t1_prev;

    auto tm = getTime();
    t1_prev = tm;

    static constexpr int jitter_threshold_usec = 300;
    ros::Time ros_tm = ros::Time::now();
    while (!is_shutdown.load() && ros::ok()) {

        ros::Time now = ros::Time::now();
        ros::Duration d_period = now - ros_tm;
        hw.read  (now, d_period);

        try {
            cm.update(now, d_period);
        }catch(const std::runtime_error &err) {
            ROS_ERROR_STREAM("plugin error:"<<err.what());
        }

        hw.write (now, d_period);
        ros_tm = now;

        getNextTime(tm, period_ns);
        sleepUntil(tm);

        auto t1 = getTime();
        auto cycle_usec = getTimeDiff_usec(t1,t1_prev);
        if(fabs(cycle_usec - period_usec) > jitter_threshold_usec){
            ROS_WARN_STREAM("jitter has ocurred : "<<(cycle_usec-period_usec)<<"[us]");
        }
        t1_prev = t1;
    }
}

int main(int argc, char **argv) {

    QApplication *app_tmp = nullptr;
    std::thread qt_thread([&]() {
        QApplication app(argc, argv);
        app_tmp = &app;
        app.exec();
    });


    ros::init(argc, argv, "seed_ros_controller");


    ros::NodeHandle root_nh;
    ros::NodeHandle my_nh("~");

    std::signal(SIGTERM, sig_handler);

    ros::AsyncSpinner spinner(3);
    spinner.start();

    setSchedPolicy(40,SCHED_RR);
    setCpuAffinity(2);

    HardwareAdaptor hw;
    hw.init(root_nh, my_nh);

    controller_manager::ControllerManager cm(&hw, root_nh);

    double control_period_sec = 0.01;
    my_nh.param("control_period", control_period_sec, control_period_sec);

    rt_loop(hw,cm,control_period_sec);

    app_tmp->quit();
    qt_thread.join();

    std::cout<<"======== seed_ros_controller has finished cleanly ========"<<std::endl;

    return 0;
}
