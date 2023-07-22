/**
 * @file log_lib.h
 * @author yang (you@domain.com)
 * @brief spdlog的头文件引用，可以使代码任意位置使用默认的 default logger
 * @version 0.1
 * @date 2023-03-28
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef LOG_LIB_H
#define LOG_LIB_H

#include <ctime>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/sinks/basic_file_sink.h" // support for basic file logging



#endif // !LOG_LIB_H