#pragma once

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/range/iterator_range.hpp>
#include <vector>
#include <unordered_map>
#include <thread>

#include "swap_buffer.hpp"

/**
 *  生の受信データを格納する構造体
 *
 */
struct Data{
    static constexpr int mlen = 256;
    int size = 0; // 格納サイズ
    int rpos = 0; // 読み込み位置
    uint8_t data[mlen] = {0};

    int maxlen(){
        return mlen;
    }
};

class SerialCommunication {
public:
    SerialCommunication();
    ~SerialCommunication();

    bool open(std::string _port, unsigned int _baud_rate);
    void close();

    int read(uint8_t* data, int len);
    int write(uint8_t* data, int len);

    bool connected();

private:
    void start_read();
    void read_thread();
    int read_impl(uint8_t* data, int len);


private:
    std::string port;
    std::atomic<bool> is_end = true;
    std::atomic<bool> is_conn = false;
    int fd = -1;
    std::thread th;
    Data* read_d = nullptr;
    SwapBuffer<Data,50> buff;

#ifdef SEED4
    uint8_t sendHeader[2];
    std::mutex mtx;
    std::condition_variable cond;
#endif
};
