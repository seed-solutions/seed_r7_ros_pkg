/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Yohei Kakiuchi (JSK lab.)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "seed_r7_ros_controller/seed_r7_robot_hardware.h"
#include "seed_r7_ros_controller/seed_r7_mover_controller.h"
#include "seed_r7_ros_controller/seed_r7_hand_controller.h"
#include "seed_r7_ros_controller/configurator.h"

#include "seed_r7_ros_controller/cosmo_cmd.h"
#include "seed_r7_ros_controller/robot_status_cmd.h"

#define NSEC_PER_SEC    1000000000L

int main(int argc, char** argv)
{
  ros::init(argc, argv, "seed_r7_ros_controller");

  ros::NodeHandle nh;
  ros::NodeHandle robot_nh("~");

  robot_hardware::RobotHW hw;
  if (!hw.init(nh, robot_nh)) {
    ROS_ERROR("Failed to initialize hardware");
    exit(1);
  }

  CosmoCmd cosmo(nh,&hw);
  RobotStatusCmd robotStatus(nh,&hw);

  //add extra controller
  robot_hardware::MoverController mover_node(nh, &hw);
  robot_hardware::HandController hand_node(robot_nh, &hw);
  robot_hardware::Configurator config_node(robot_nh, &hw);

  ros::AsyncSpinner spinner(5);
  spinner.start();

  double period = hw.getPeriod();
  controller_manager::ControllerManager cm(&hw, nh);

  ROS_INFO("ControllerManager start with %f Hz", 1.0/period);
  // TODO: realtime loop

  int cntr = 0;
  long main_thread_period_ns = period*1000*1000*1000;
  double max_interval = 0.0;
  double ave_interval = 0.0;
  timespec m_t;
  clock_gettime(CLOCK_MONOTONIC, &m_t);

  ros::Time ros_tm = ros::Time::now();
  while (ros::ok()) {
    {
      static timespec tm;
      tm.tv_nsec = m_t.tv_nsec;
      tm.tv_sec  = m_t.tv_sec;
      tm.tv_nsec += main_thread_period_ns;
      while (tm.tv_nsec >= NSEC_PER_SEC) {
        tm.tv_nsec -= NSEC_PER_SEC;
        ++tm.tv_sec;
      }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tm, NULL);


      if (cntr > 100) {
        ROS_INFO("max: %f [ms], ave: %f [ms]", max_interval/1000, ave_interval/1000);
        cntr = 0;
        max_interval = 0.0;
      }
      static timespec n_t;
      clock_gettime(CLOCK_MONOTONIC, &n_t);
      const double measured_interval
        = ((n_t.tv_sec - m_t.tv_sec)*NSEC_PER_SEC + (n_t.tv_nsec - m_t.tv_nsec))/1000.0;  // usec
      if (measured_interval > max_interval) max_interval = measured_interval;
      if (ave_interval == 0.0) {
        ave_interval = measured_interval;
      } else {
        ave_interval = (measured_interval + (100 - 1)*ave_interval)/100.0;
      }
      m_t.tv_sec  = n_t.tv_sec;
      m_t.tv_nsec = n_t.tv_nsec;
      ++cntr;
    }

    ros::Time now = ros::Time::now();
    ros::Duration d_period = now - ros_tm;
    hw.read  (now, d_period);
    cm.update(now, d_period);
    hw.write (now, d_period);
    ros_tm = now;
  }

  return 0;
}
