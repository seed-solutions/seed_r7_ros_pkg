#pragma once

template<class T,int N>
struct is_byte{
    static const bool value = sizeof(T) == N;
};
