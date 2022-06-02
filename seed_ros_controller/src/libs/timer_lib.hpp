#pragma once

#include <ctime>

static constexpr long NSEC_PER_SEC = 1000000000L;
static constexpr long USEC_PER_SEC = 1000000L;
static constexpr double USEC_PER_NSEC = 1./1000;


void getNextTime(timespec &tm, long intvl_ns) {
    tm.tv_nsec += intvl_ns;
    while (tm.tv_nsec >= NSEC_PER_SEC) {
        tm.tv_nsec -= NSEC_PER_SEC;
        ++tm.tv_sec;
    }
}

void sleepUntil(timespec &tm) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tm, NULL);
}

double getTimeDiff_usec(timespec tm_now, timespec tm_prev) {
    return (tm_now.tv_sec - tm_prev.tv_sec) * USEC_PER_SEC + (tm_now.tv_nsec - tm_prev.tv_nsec) * USEC_PER_NSEC;
}

timespec getTime(){
    timespec tm_now;
    clock_gettime(CLOCK_MONOTONIC, &tm_now);
    return tm_now;
}

void setCpuAffinity(int cpu){
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(cpu, &cpu_set);
    pid_t tid = gettid();
    sched_setaffinity(tid, sizeof(cpu_set_t), &cpu_set);

}

bool setSchedPolicy(int priority,int policy){
    struct sched_param sp;
    sp.sched_priority = priority;
    if(sched_setscheduler(0,policy,&sp) == -1){
        ROS_ERROR_STREAM("sched policy cannot set. .");
        return false;
    }
    return true;
}
