#pragma once

#include<mutex>

#define DATA_TYPE_PTR
//#define DATA_TYPE_SHARED_PTR

//リアルタイムスレッド同士の通信バッファ
template<class T,int bufSize>
class SwapBuffer{
public:
    SwapBuffer()
    {
#ifdef DATA_TYPE_SHARED_PTR
        for(int idx = 0;idx<bufSize;++idx){
            data_a[idx] = std::make_shared<T>();
        }
        data_b = std::make_shared<T>();
#elif defined(DATA_TYPE_PTR)
        for(int idx = 0;idx<bufSize;++idx){
            data_a[idx] = new T();
        }
        data_b = new T();
#endif
    }

    ~SwapBuffer(){
#ifdef DATA_TYPE_SHARED_PTR

#elif defined(DATA_TYPE_PTR)
        for(int idx = 0;idx<bufSize;++idx){
            delete data_a[idx];
            data_a[idx] = nullptr;
        }
        delete data_b;
        data_b = nullptr;
#endif
    }

    //バッファ内全データクリア
    void clear(){
        pread_mtx.lock();
        while (1) {
            auto guard = std::lock_guard < std::mutex > (mtx[pread]);
            if (new_data_available[pread]) {
                new_data_available[pread] = false;
                pread = getNextPos(pread);
            }else{
                break;
            }
        }
        pread_mtx.unlock();
    }

    //読み出し可否設定
    void setPermissionRead(bool readable){
        auto guard = std::lock_guard < std::mutex > (pread_mtx);
        read_permitted = readable;
    }


    //バッファからの読み出し
    //clear中や、他スレッドが読み出している時など、読み出せない場合はnullが返る
    //wait_writeをtrueにすると、書き込み側の処理待ってから読み出す
    T* readFromB(bool wait_write = false){
        auto rp_guard = std::unique_lock < std::mutex > (pread_mtx, std::try_to_lock);
        if (!rp_guard.owns_lock() || !read_permitted) {
            return nullptr;
        }

        std::unique_lock < std::mutex > guard;
        if (!wait_write) {
            guard = std::unique_lock < std::mutex > (mtx[pread], std::try_to_lock);
        } else {
            guard = std::unique_lock < std::mutex > (mtx[pread]);
        }
        if (guard.owns_lock()) {
            //読み込み可能な領域であれば、読み込む
            if (new_data_available[pread]) {
#ifdef DATA_TYPE_PTR
                swap(&data_a[pread], &data_b);
#else
                data_b = data_a[pread];
#endif
                new_data_available[pread] = false;
                pread = getNextPos(pread);

#ifdef DATA_TYPE_SHARED_PTR
                return data_b.get();
#elif defined(DATA_TYPE_PTR)
                return data_b;
#else
                return &data_b;
#endif
            }
        }
        return nullptr;
    }

    bool writeFromA(const T& data) {
        std::lock_guard < std::mutex > pw_guard(pwrite_mtx);
        std::lock_guard < std::mutex > guard(mtx[pwrite]);

        //書き込み可能な領域であれば、書き込む
        if (!new_data_available[pwrite]) {
#ifdef DATA_TYPE_PTR
            *(data_a[pwrite]) = data;
#else
            data_a[pwrite] = data;
#endif
            new_data_available[pwrite] = true;
            pwrite = getNextPos(pwrite);
            return true;
        }
        return false;
    }

private:
    int getNextPos(int pos){
        pos++;
        if(pos >= bufSize){
            pos = 0;
        }
        return pos;
    }


    template<class U>
    void swap(U *a, U *b) {
        U tmp = *a;
        *a = *b;
        *b = tmp;
    }

private:
    std::mutex rp_mtx;
    std::mutex pread_mtx;
    std::mutex pwrite_mtx;

    bool read_permitted = true;

    int pread = 0;
    int pwrite = 0;
#ifdef DATA_TYPE_SHARED_PTR
    std::shared_ptr<T> data_a[bufSize] = {nullptr};
    std::shared_ptr<T> data_b = nullptr;
#elif defined(DATA_TYPE_PTR)
    T* data_a[bufSize] = {nullptr};
    T* data_b = nullptr;
#else
    T data_a[bufSize];
    T data_b;
#endif
    bool new_data_available[bufSize] = {false};
    std::mutex mtx[bufSize];
};
