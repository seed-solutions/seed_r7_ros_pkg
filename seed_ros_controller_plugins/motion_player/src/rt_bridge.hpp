#pragma once

#include <vector>
#include <libs/swap_buffer.hpp>
#include "aero_data.hpp"

namespace motion_player {

struct Request{
    int msid;
    int step;
    BuffRaw rawdata;
};

struct Response{
    int step = 0;
    bool done = false;
};

static constexpr int REQ_BUFF_SIZE = 50;
static constexpr int RESP_BUFF_SIZE = 20;

using reqBuffType = SwapBuffer<Request, REQ_BUFF_SIZE>;
using respBuffType = SwapBuffer<Response, RESP_BUFF_SIZE>;

}
