#include "common.hpp"
#include <iomanip>

//古いコンパイラでは未実装なので、自前で用意
#if __GNUC__ <= 7
namespace std {
namespace experimental {
namespace filesystem {
path relative(path p, path base) {
    p = absolute(p);
    base = absolute(base);
    auto mismatched = std::mismatch(p.begin(), p.end(), base.begin(), base.end());
    if (mismatched.first == p.end() && mismatched.second == base.end())
        return ".";

    auto it_p = mismatched.first;
    auto it_base = mismatched.second;

    path ret;

    for (; it_base != base.end(); ++it_base)
        ret /= "..";

    for (; it_p != p.end(); ++it_p)
        ret /= *it_p;

    return ret;
}
}
}
}

#endif

std::string getTimeStamp()
{
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(localtime(&now_c), "[%Y/%m/%d %H:%M:%S]");
    return ss.str();
}
