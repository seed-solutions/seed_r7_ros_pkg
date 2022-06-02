#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include "other_cmd_buff.hpp"

namespace hardware_interface {

class OtherCommandHandle {
public:
    OtherCommandHandle() = default;

    OtherCommandHandle(const int msid, BuffList *buffList_recv, BuffList *buffList_send) :
            msid(msid), buffList_recv(buffList_recv), buffList_send(buffList_send) {
    }

    bool setSendCmd(BuffRaw* buf){
        if(buffList_send->size >= buffList_send->capacity){
            ROS_ERROR_STREAM("Capacity over. This command will not be sent to the robot.");
            return false;
        }
        memcpy(&buffList_send->buf[buffList_send->size],buf,sizeof(BuffRaw));
        buffList_send->size++;
        return true;
    }

    void getCmd(uint8_t header1,uint8_t header2,BuffList *list){
        if(!buffList_recv){
            return;
        }

        int idx2 = list->size;
        for (int idx = 0; idx < buffList_recv->size; ++idx) {
            if (buffList_recv->buf[idx].data[0] == header1 && buffList_recv->buf[idx].data[1] == header2) {
                memcpy(&list->buf[idx2], &buffList_recv->buf[idx], sizeof(BuffRaw));
                list->size = idx2 + 1;
                idx2++;
            }
        }

        return;
    }


    std::string getName() const {
        return std::to_string(msid);
    }

private:
    int msid = 0;
    BuffList *buffList_recv = nullptr;
    BuffList *buffList_send = nullptr;
};

class OtherCommandInterface: public HardwareResourceManager<OtherCommandHandle> {
};

}

