#include "serial_com.hpp"

#include <linux/serial.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

#include "debug_tools.hpp"

///////////////////////////////
SerialCommunication::SerialCommunication() {
}

///////////////////////////////
SerialCommunication::~SerialCommunication() {
    close();
}

///////////////////////////////
bool SerialCommunication::open(std::string port, unsigned int baud_rate) {
    this->port = port;

    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC); //端末関連の制御を無効 writeを、相手に届くまで待つようにする
    if (fd < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return false;
    }

    struct termios tio; // シリアル通信設定
    if(tcgetattr(fd, &tio) != 0) {// 現在の設定を拾ってくる
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    cfmakeraw(&tio);//特殊文字の制御などの機能を切る

    //入出力それぞれのボーレート
    speed_t brate = B38400;
    if(baud_rate == 1000000){
        brate = B1000000;
    }else{
        printf("Error selected baud rate is not supported. :%d\n",baud_rate);
        return false;
    }
    cfsetispeed( &tio, brate );
    cfsetospeed( &tio, brate );


    tio.c_cc[VTIME] = 10;//*0.1[s] readのタイムアウト時間
    tio.c_cc[VMIN] = 0;//読み込んだとみなす最小文字数

    tcsetattr( fd, TCSANOW, &tio );//すぐ反映モードで設定    ioctl(fd, TCSETS, &tio)と同じ

    //低遅延モードにする
    struct serial_struct serial_setting;
    ioctl(fd, TIOCGSERIAL, &serial_setting);
    serial_setting.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &serial_setting);

    is_end.store(false);
    is_conn.store(true);

    //一応、バッファをフラッシュしておく
    tcflush(fd, TCIOFLUSH);
    start_read();


    return true;
}

void SerialCommunication::close() {
    is_end.store(true);

    if(th.joinable()){
        th.join();
    }

    ::close(fd);
    is_conn.store(false);
}

bool SerialCommunication::connected(){
    return is_conn.load();
}

void SerialCommunication::start_read(){
    th = std::thread ( &SerialCommunication::read_thread,this );
}

void SerialCommunication::read_thread() {
    while (!is_end.load()) {
        Data d;
        d.size = read_impl(d.data, d.maxlen());
        if (d.size > 0) {
            buff.writeFromA(d);
        }
    }
}

int SerialCommunication::write(uint8_t* data, int len) {
    if(fd < 0 || !is_conn.load()){
        return 0;
    }

    int writed_size = ::write(fd, data, len);
    if(writed_size == -1){
        printf("write error: [Errno %d] %s\n", errno, strerror(errno));
        if(errno == EBADF || errno == EIO || errno == ENODEV){
            is_conn.store(false);
        }
    }


#include "mode.hpp"
#ifdef OTAMA
    usleep(2000);
#endif


    return writed_size;
}

int SerialCommunication::read(uint8_t* data, int len){
    if (!read_d || read_d->size == read_d->rpos) {
        read_d = buff.readFromB();
        if(!read_d){
            return 0;
        }
    }

    int read_len = read_d->size - read_d->rpos;
    if(len < read_len){
        read_len = len;
    }

    memcpy(data,read_d->data+read_d->rpos,read_len);
    read_d->rpos += read_len;

    return read_len;
}

int SerialCommunication::read_impl(uint8_t* data, int len){
    if(fd < 0 || !is_conn.load()){
        return 0;
    }

    int readed_size = ::read(fd, data, len);
    if(readed_size == -1){
        printf("read error: [Errno %d] %s\n", errno, strerror(errno));
        if(errno == EBADF || errno == EIO || errno == ENODEV){
            is_conn.store(false);
        }
    }

//    if(readed_size > 0)dump(("read "+port).c_str(),data,readed_size);

    return readed_size;
}
