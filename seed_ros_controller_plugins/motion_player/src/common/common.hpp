#pragma once

#ifdef WIN32
#include <winsock2.h>
#define _USE_MATH_DEFINES
#include <math.h>
#endif

#include<cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <list>
#include <vector>
#include <algorithm>
#include <string>
#include <thread>
#include <chrono>
#include <regex>
#include <memory>
#include <mutex>
#include <map>
#include <unordered_map>
#include <typeinfo>
#include <queue>
#include <condition_variable>
#include <typeindex>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <pthread.h>
#include <cstdlib>
#include <cstring>
#include <atomic>

#if __GNUC__ <= 7
#include <experimental/filesystem>

namespace std {
namespace experimental {
namespace filesystem {
path relative(path p, path base);
}
}
namespace filesystem = experimental::filesystem;
}

#else
#include <filesystem>
#endif

std::string getTimeStamp();

#ifdef __linux__

#ifdef DEBUG
#define LOG_ERROR_STREAM(str) std::cout<<getTimeStamp()<<"\033[31m[ERROR][pid:"<<getpid()<<"][thid:"<<(void*)pthread_self()<<"][func:"<<__FUNCTION__<<"] "<< str
#define LOG_TRACE_STREAM(str) std::cout<<getTimeStamp()<<"[TRACE][pid:"<<getpid()<<"][thid:"<<(void*)pthread_self()<<"][func:"<<__FUNCTION__<<"] "<< str
#define LOG_INFO_STREAM(str) std::cout<<getTimeStamp()<<"\x1b[36m[INFO][pid:"<<getpid()<<"][thid:"<<(void*)pthread_self()<<"][func:"<<__FUNCTION__<<"] "<< str
#define LOG_DEBUG_STREAM(str) std::cout<<getTimeStamp()<<"[DEBUG][pid:"<<getpid()<<"][thid:"<<(void*)pthread_self()<<"][func:"<<__FUNCTION__<<"] "<< str
#define LOG_STREAM_CONTINUE(str) std::cout<<str
#define LOG_END std::dec<<"\033[m"<<std::endl
#else
#define LOG_ERROR_STREAM(str) std::cout<<getTimeStamp()<<"\033[31m[ERROR][pid:"<<getpid()<<"][thid:"<<(void*)pthread_self()<<"][func:"<<__FUNCTION__<<"] "<< str
#define LOG_TRACE_STREAM(str)
#define LOG_INFO_STREAM(str) std::cout<<getTimeStamp()<<"\x1b[36m[INFO][pid:"<<getpid()<<"][thid:"<<(void*)pthread_self()<<"][func:"<<__FUNCTION__<<"] "<< str
#define LOG_DEBUG_STREAM(str)
#define LOG_STREAM_CONTINUE(str) std::cout<<str
#define LOG_END std::dec<<"\033[m"<<std::endl
#endif

#else

#include <windows.h>

#ifdef DEBUG
#define LOG_ERROR_STREAM(str) std::cout<<getTimeStamp()<<"\033[31m[ERROR][thid:"<<(void*)pthread_self()<<"][func:"<<__FUNCTION__<<"] "<< str
#define LOG_TRACE_STREAM(str) std::cout<<getTimeStamp()<<"[TRACE][thid:"<<(void*)pthread_self()<<"][func:"<<__FUNCTION__<<"] "<< str
#define LOG_INFO_STREAM(str) std::cout<<getTimeStamp()<<"\x1b[36m[INFO][thid:"<<(void*)pthread_self()<<"][func:"<<__FUNCTION__<<"] "<< str
#define LOG_DEBUG_STREAM(str) std::cout<<getTimeStamp()<<"[DEBUG][thid:"<<(void*)pthread_self()<<"][func:"<<__FUNCTION__<<"] "<< str
#define LOG_STREAM_CONTINUE(str) std::cout<<str
#define LOG_END std::dec<<"\033[m"<<std::endl
#else
#define LOG_ERROR_STREAM(str) std::cout<<getTimeStamp()<<"\033[31m[ERROR][thid:"<<GetCurrentThreadId()<<"][func:"<<__FUNCTION__<<"] "<< str
#define LOG_TRACE_STREAM(str)
#define LOG_INFO_STREAM(str) std::cout<<getTimeStamp()<<"\x1b[36m[INFO][thid:"<<GetCurrentThreadId()<<"][func:"<<__FUNCTION__<<"] "<< str
#define LOG_DEBUG_STREAM(str)
#define LOG_STREAM_CONTINUE(str) std::cout<<str
#define LOG_END std::dec<<"\033[m"<<std::endl
#endif

#endif
