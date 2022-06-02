#pragma once

#include "common.hpp"
#include "utilities.hpp"

typedef std::vector<std::string> args_type;

template<class ...T>
bool get_value_impl(args_type::const_iterator itr, args_type::const_iterator itr_end, T ...) {
    return true;
}


template<class HEAD, class ... Tail>
bool get_value_impl(args_type::const_iterator itr, args_type::const_iterator itr_end, HEAD *head, Tail ... tail) {

    if (itr == itr_end) return false;

    if(!from_stdstring(*itr,head)){
        return false;
    }

    return get_value_impl(++itr, itr_end, tail...);
}

template<class ... T>
bool get_value(const args_type &vars, T ... value) {
    return get_value_impl(vars.begin(), vars.end(), value...);
}

class Extension{
public:
    virtual ~Extension() = default;

    virtual std::string get_name() = 0;

    virtual void entry(){};

    virtual void exit(){};

    virtual bool execute(const args_type &vars) = 0;

    typedef Extension* Ptr;

};
