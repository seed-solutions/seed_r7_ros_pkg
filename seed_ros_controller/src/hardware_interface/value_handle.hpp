#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>


namespace hardware_interface{

template<class T>
class ValueHandle
{
public:
    ValueHandle() = default;

    ValueHandle(const std::string name, T* cmd)
    : joint_name(name), cmd_(cmd)
  {
    if (!cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + name + "'. Command data pointer is null.");
    }
  }

  void setCommand(T command) {assert(cmd_); *cmd_ = command;}
  T getCommand() const {assert(cmd_); return *cmd_;}
  const T* getCommandPtr() const {assert(cmd_); return cmd_;}

  std::string getName() const{
      return joint_name;
  }

private:
  std::string joint_name;
  T* cmd_ = nullptr;
};
}

