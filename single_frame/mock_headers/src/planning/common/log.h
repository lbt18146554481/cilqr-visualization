/*
 * Mock header for logging macros
 */

#pragma once

#include <iostream>
#include <sstream>

#define LOG_INFO(...) do { \
    std::ostringstream oss; \
    oss << "[INFO] " << __VA_ARGS__; \
    std::cout << oss.str() << std::endl; \
} while(0)

#define LOG_ERROR(...) do { \
    std::ostringstream oss; \
    oss << "[ERROR] " << __VA_ARGS__; \
    std::cerr << oss.str() << std::endl; \
} while(0)

#define LOG_WARNING(...) do { \
    std::ostringstream oss; \
    oss << "[WARNING] " << __VA_ARGS__; \
    std::cerr << oss.str() << std::endl; \
} while(0)

#define LOG_WARN(...) LOG_WARNING(__VA_ARGS__)

#define LOG_DEBUG(...) do { \
    std::ostringstream oss; \
    oss << "[DEBUG] " << __VA_ARGS__; \
    std::cout << oss.str() << std::endl; \
} while(0)

// Mock for INIT_NEOLOG_NAME
#define INIT_NEOLOG_NAME(name) do { \
    (void)(name); \
} while(0)
