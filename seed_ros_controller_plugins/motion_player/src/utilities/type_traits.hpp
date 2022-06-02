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

template<class T1,class T2>
struct is_derived_from{
    static const bool value = (!std::is_same<T2,T1>::value) && (std::is_base_of<T2,T1>::value);
};

template<class T1, class T2>
struct is_exportable {

    template<class T3, class T4>
    static auto check(T3 lhs, T4 rhs) -> decltype(lhs<<rhs,std::true_type());
    static auto check(...) -> std::false_type;

    static constexpr bool value = decltype(check(std::declval<T1>(),std::declval<T2>()))::value;
};

template<class T>
struct has_iterator{
    template<class T1>
    static constexpr std::true_type check(typename T1::iterator* val);

    template<class T1>
    static constexpr std::false_type check(...);

    static constexpr bool value = decltype(check<T>(nullptr))::value;
};

template<class T>
struct is_pair{
    template<class T1>
    static constexpr bool check(decltype(std::declval<T1>().first)* val){return true;};

    template<class T1>
    static constexpr bool check(...){return false;};

    static constexpr bool value = check<T>(nullptr);
};
