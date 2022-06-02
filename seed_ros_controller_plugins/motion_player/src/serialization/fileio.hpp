#pragma once

#include "archive.hpp"

class access{
public:
    template<class T1,class T2>
    void save(const T1& target,T2& format){
        Archive<T2> ar(Archive<T2>::DIRECTION_OUTPUT,format);
        const_cast<T1*>(&target)->serialize(ar);
    }

    template<class T1,class T2>
    void load(const T1& target,T2& format){
        Archive<T2> ar(Archive<T2>::DIRECTION_INPUT,format);
        const_cast<T1*>(&target)->serialize(ar);
    }

    template<class T1,class T2>
    void save(T1& target,T2& format){
        Archive<T2> ar(Archive<T2>::DIRECTION_OUTPUT,format);
        target.serialize(ar);
    }

    template<class T1,class T2>
    void load(T1& target,T2& format){
        Archive<T2> ar(Archive<T2>::DIRECTION_INPUT,format);
        target.serialize(ar);
    }

};
