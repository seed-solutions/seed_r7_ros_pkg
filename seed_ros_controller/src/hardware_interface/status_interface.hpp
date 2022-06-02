#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include "value_handle.hpp"
#include "robot_status.hpp"

namespace hardware_interface{
using StatusHandle = ValueHandle<Status>;
class StatusInterface : public HardwareResourceManager<StatusHandle> {};
}

