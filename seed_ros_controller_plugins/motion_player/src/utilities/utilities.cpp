#include "utilities.hpp"


void cyclicsleep(int cycle_ns,struct timespec &time){
    if (time.tv_nsec == 0 && time.tv_sec == 0) {
        clock_gettime(CLOCK_MONOTONIC, &time);
    } else {

        time.tv_nsec += cycle_ns;
        if (time.tv_nsec >= SEC_NSSCALE) {
            time.tv_nsec -= SEC_NSSCALE;
            time.tv_sec += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time, NULL);
    }
    return;
}

void millisleep(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

std::string replaceString(std::string src, std::string tgt, std::string str) {
    std::string::size_type pos(src.find(tgt));
    src.replace(pos, tgt.length(), str);

    return src;
}

std::vector<std::string> split(const std::string &src, const char *delim) {
    std::vector < std::string > vec;
    std::string::size_type len = src.length();

    for (std::string::size_type i = 0, n; i < len; i = n + 1) {
        n = src.find_first_of(delim, i);
        if (n == std::string::npos) {
            n = len;
        }
        if (n != i) {
            vec.push_back(src.substr(i, n - i));
        }
    }

    return vec;
}

bool dbl_equal(double a, double b) {
    return fabs(a - b) <= std::numeric_limits<double>::epsilon() * fmax(1, fmax(fabs(a), fabs(b)));
}

bool dbl_equal(double a, double b, double tol) {
    return fabs(a - b) <= tol;
}

bool from_stdstring(const std::string &str, std::string *data) {
    *data = str;
    return true;
}

bool from_stdstring(const std::string &str, int *data) {
    *data = std::stoi(str);
    return true;
}

bool from_stdstring(const std::string &str, double *data) {
    *data = std::stod(str);
    return true;
}

std::string getEnvStr(std::string name, std::string defval) {
    char const *tmp = getenv(name.c_str());
    std::string ret = (tmp == nullptr) ? defval : std::string(tmp);
    return ret;
}

double restrictRotation(double deg) {

    deg = fmod(deg, 360.);

    if (deg > 180) {
        deg = -180. + fmod(deg, 180.);
    } else if (deg <= -180.) {
        deg = 180. + fmod(deg, 180.);
    }
    return deg;
}

double restrictRotationRad(double rad) {

    rad = fmod(rad, 2 * M_PI);

    if (rad > M_PI) {
        rad = -M_PI + fmod(rad, M_PI);
    } else if (rad <= -M_PI) {
        rad = M_PI + fmod(rad, M_PI);
    }
    return rad;
}

double round_n(double number, int n)
{
    if(n <= 1){
        return round(number);
    }

    int amp = pow(10,n-1);
    return round(number*amp)/amp;
}

