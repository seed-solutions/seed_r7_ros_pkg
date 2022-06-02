#pragma once

#include "common.hpp"

#define ARCHIVE_NAMEDVALUE(value) createNamedValue(#value,value)

template<class T>
struct KeyedValue{
    std::string key;
    T &value;
};

template<class T>
KeyedValue<T> createNamedValue(std::string key,T& value){
    return KeyedValue<T>{key,value};
}

template<class Format>
struct Archive{
    enum e_DIRECTION{
        DIRECTION_INPUT,
        DIRECTION_OUTPUT,
    };

    Archive(e_DIRECTION direction,Format& f):direction(direction),f(f){
    }

    e_DIRECTION getDirection(){
        return direction;
    }

    template<class T>
    void operator&(T& val){
        KeyedValue<T> keyedval;
        keyedval.key = "value"+(vno++);
        keyedval.value = val;
        operator&(keyedval);
    }

    template<class T>
    void operator&(KeyedValue<T> keyedval){
        if (direction == DIRECTION_INPUT) {
            operator>>(keyedval);
        } else {
            operator<<(keyedval);
        }
    }

    template<class T>
    void operator<<(const T& val){
        KeyedValue<T> keyedval;
        keyedval.key = "value"+(vno++);
        keyedval.value = val;
        operator<<(keyedval);
    }

    template<class T>
    void operator<<(const KeyedValue<T> keyedval){
        if (direction == DIRECTION_INPUT) {
            //入力モード時は、何もしない
            return;
        }

        f << keyedval;
    }

    template<class T>
    void operator>>(T& val){
        KeyedValue<T> keyedval;
        keyedval.key = "value"+(vno++);
        keyedval.value = val;
        operator>>(keyedval);
    }

    template<class T>
    void operator>>(KeyedValue<T> keyedval){
        if (direction == DIRECTION_OUTPUT) {
            //出力モード時は、何もしない
            return;
        }

        f >> keyedval;
    }


private:
    e_DIRECTION direction;
    int vno = 0;

    std::stringstream ss;
    Format& f;

};
