#pragma once

#include "cmd_data.hpp"

#include <mutex>
#include <unistd.h>

int open_pseudo_serial(const char *dev_path, int *mfd, int *sfd);

class Serial{
public:
    Serial(){
    }

    ~Serial(){
        close();
    }


    bool open(const std::string& dev_path){
        this->dev_path = dev_path;

        bool is_open = (open_pseudo_serial(dev_path.c_str(),&master,&slave) == 0);

        if (is_open) {
            maxfd = master;
            FD_ZERO(&readfds);
            FD_SET(master, &readfds);
        }

        return is_open;
    }

    std::string getport(){
        return dev_path;
    }

    void close(){
        if (slave > 0) {
            ::close(slave);
            slave = -1;
        }

        if (master > 0) {
            ::close(master);
            master = -1;
        }

        ::remove(dev_path.c_str());
    }

    void write(void *data,int len){
        write_mtx.lock();
        ::write(master, data, len);
        write_mtx.unlock();
    }

    bool read(void *data, int len){
        bool ret = false;
        int r;

        read_mtx.lock();

        memcpy(&fds, &readfds, sizeof(fd_set));
        tv_to.tv_sec = 1;
        tv_to.tv_usec = 0;
        select(maxfd + 1, &fds, NULL, NULL, &tv_to);
        if (FD_ISSET(master, &fds)) {
            if ((r = ::read(master, data, len)) > 0) {
                ret = true;
            }
        }
        read_mtx.unlock();

        return ret;
    }

private:
    fd_set fds, readfds;
    int maxfd = -1;

    struct timeval tv_to;

    std::string dev_path;

    int master = -1;
    int slave = -1;

    std::mutex read_mtx;
    std::mutex write_mtx;
};
