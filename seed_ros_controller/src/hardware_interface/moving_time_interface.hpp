#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include "value_handle.hpp"

namespace hardware_interface {
using MovingTimeHandle = ValueHandle<double>;
class MovingTimeInterface: public HardwareResourceManager<MovingTimeHandle> {
};
}

