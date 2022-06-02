#pragma once

#include "common.hpp"

namespace std {
template<bool T1, class T2 = void>
struct disable_if {
};

template<class T2>
struct disable_if<false, T2> {
    typedef T2 type;
};
}


//イテレータを持っているかどうかをチェックする。
template<class T>
struct has_iterator{
    template<class T1>
    static constexpr std::true_type check(typename T1::iterator* val);

    template<class T1>
    static constexpr std::false_type check(...);

    static constexpr bool value = decltype(check<T>(nullptr))::value;
};


template<class T,int N>
struct is_byte{
    static const bool value = sizeof(T) == N;
};
