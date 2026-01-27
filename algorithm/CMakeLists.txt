cmake_minimum_required(VERSION 3.10)
project(CILQRAlgorithm)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找Eigen3库
find_package(Eigen3 QUIET)
if(NOT EIGEN3_INCLUDE_DIR AND NOT EIGEN3_INCLUDE_DIRS)
    set(EIGEN3_INCLUDE_DIR "")
    file(GLOB EIGEN3_CELLAR_PATHS "/opt/homebrew/Cellar/eigen/*/include/eigen3")
    if(EIGEN3_CELLAR_PATHS)
        list(GET EIGEN3_CELLAR_PATHS 0 EIGEN3_INCLUDE_DIR)
        message(STATUS "Found Eigen3 in Cellar: ${EIGEN3_INCLUDE_DIR}")
    else()
        set(EIGEN3_POSSIBLE_PATHS
            "/opt/homebrew/include/eigen3"
            "/usr/local/include/eigen3"
            "/usr/include/eigen3"
        )
        foreach(PATH ${EIGEN3_POSSIBLE_PATHS})
            if(EXISTS ${PATH})
                set(EIGEN3_INCLUDE_DIR ${PATH})
                message(STATUS "Found Eigen3 at: ${EIGEN3_INCLUDE_DIR}")
                break()
            endif()
        endforeach()
    endif()
endif()

if(EIGEN3_INCLUDE_DIRS AND NOT EIGEN3_INCLUDE_DIR)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Eigen3 not found! Please install it: brew install eigen")
else()
    message(STATUS "Using Eigen3 at: ${EIGEN3_INCLUDE_DIR}")
endif()

# 包含目录
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

# 查找spdlog（用于日志）
find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
    # 如果找不到，尝试从根目录的3rdparty目录查找
    if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../3rdparty/spdlog")
        set(SPDLOG_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../3rdparty/spdlog/include")
        include_directories(${SPDLOG_INCLUDE_DIR})
        message(STATUS "Using spdlog from parent project")
    endif()
endif()

# 收集源文件
file(GLOB SOURCE_FILES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")

# 创建算法库
add_library(cilqr_algorithm STATIC ${SOURCE_FILES})

target_compile_features(cilqr_algorithm PUBLIC cxx_std_17)

# 设置输出目录
set_target_properties(cilqr_algorithm PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

# 安装
install(TARGETS cilqr_algorithm
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)
