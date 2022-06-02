#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include "value_handle.hpp"

namespace hardware_interface {
using ScriptHandle = ValueHandle<uint16_t>;
class JointScriptInterface: public HardwareResourceManager<ScriptHandle> {
};
}

