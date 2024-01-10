#include <regex>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream> //iostream file
#include <ros/package.h>

#include "seed_r7_ros_controller/RobotInfo.h"

//////////////////////////////
class RecordRobotInfo
{
public:
  RecordRobotInfo(ros::NodeHandle _nh);
  void GetRobotInfo(const seed_r7_ros_controller::RobotInfoConstPtr &_data);

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::string pkg_path_;
  std::string file_path_;

  ros::Time start_time_;
  int sec;
};

RecordRobotInfo::RecordRobotInfo(const ros::NodeHandle _nh) : nh_(_nh)
{
  file_path_ = std::string(std::getenv("HOME")) + "/Desktop/Record_Data.csv";
  sub_ = nh_.subscribe("/seed_r7_ros_controller/robot_info", 1, &RecordRobotInfo::GetRobotInfo, this);
  pkg_path_ = ros::package::getPath("seed_r7_samples");
  std::ofstream ofs_(file_path_, std::ios_base::trunc); // delte file

  // 文字化け対策のため、BOM追加
  unsigned char bom[] = {0xEF, 0xBB, 0xBF};
  ofs_.write((char *)bom, sizeof(bom));

  start_time_ = ros::Time::now();
  ofs_ << "経過時間,バッテリ,";
  ofs_ << "指令速度(x), 指令速度(y), 指令速度(theta), ";
  ofs_ << "実速度(x), 実速度(y), 実速度(theta), ";
  ofs_ << "実位置(x), 実位置(y), 実位置(theta), ";
  ofs_ << "位置(右前),位置(右後),位置(左前),位置(左後),";
  ofs_ << "電流(右前),電流(右後),電流(左前),電流(左後),";
  ofs_ << "温度(右前),温度(右後),温度(左前),温度(左後),";
  ofs_ << "状態(右前),状態(右後),状態(左前),状態(左後),状態(全体)";

  ofs_ << std::endl;
}

void RecordRobotInfo::GetRobotInfo(const seed_r7_ros_controller::RobotInfoConstPtr &_data)
{
  std::ofstream ofs_(file_path_, std::ios_base::app); // add data
  ofs_ << (ros::Time::now() - start_time_).toSec() << "," << _data->robot.voltage << ",";
  ofs_ << _data->robot.cmd_vel.x << "," << _data->robot.cmd_vel.y << "," << _data->robot.cmd_vel.theta << ",";
  ofs_ << _data->robot.odom_vel.x << "," << _data->robot.odom_vel.y << "," << _data->robot.odom_vel.theta << ",";
  ofs_ << _data->robot.odom_pos.x << "," << _data->robot.odom_pos.y << "," << _data->robot.odom_pos.theta << ",";
  ofs_ << _data->driver[0].position << "," << _data->driver[1].position << "," << _data->driver[2].position << "," << _data->driver[3].position << ",";
  ofs_ << _data->driver[0].current << "," << _data->driver[1].current << "," << _data->driver[2].current << "," << _data->driver[3].current << ",";
  ofs_ << _data->driver[0].temp << "," << _data->driver[1].temp << "," << _data->driver[2].temp << "," << _data->driver[3].temp << ",";
  ofs_ << _data->driver[0].status << "," << _data->driver[1].status << "," << _data->driver[2].status << "," << _data->driver[3].status << ",";
  ofs_ << _data->robot.status.c_str();
  ofs_ << std::endl;

  ROS_INFO("record robot info");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "record_robot_info_node");
  ros::NodeHandle nh;

  RecordRobotInfo record(nh);

  ros::spin();

  return 0;
}
