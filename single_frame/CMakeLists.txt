
cmake_minimum_required(VERSION 3.10)
project(PlanningVisual)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 优先使用系统的 jsoncpp（arm64），如果找不到再使用本地的
find_package(jsoncpp QUIET)
if(jsoncpp_FOUND)
    message(STATUS "Using system jsoncpp")
    set(JSONCPP_INCLUDE_DIRS ${jsoncpp_INCLUDE_DIRS})
    set(JSONCPP_LIBRARIES jsoncpp_lib)
else()
    # 尝试使用 Homebrew 安装的 jsoncpp
    if(EXISTS /opt/homebrew/lib/libjsoncpp.dylib)
        set(JSONCPP_INCLUDE_DIRS /opt/homebrew/include)
        set(JSONCPP_LIBRARIES /opt/homebrew/lib/libjsoncpp.dylib)
        message(STATUS "Using Homebrew jsoncpp: ${JSONCPP_LIBRARIES}")
    else()
        # 使用本地 jsoncpp（x86_64，可能不兼容 arm64）
        set(JSONCPP_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/jsoncpp_install/include")
        set(JSONCPP_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/jsoncpp_install/lib/libjsoncpp.a")
        message(WARNING "Using local jsoncpp (may have architecture mismatch)")
    endif()
endif()

# 添加本地jsoncpp头文件和缺失头文件目录
include_directories(
    ${JSONCPP_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/mock_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/mock_headers/config
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)




# Eigen3查找
find_package(Eigen3 QUIET)

# 如果找不到Eigen3，尝试手动设置
if(NOT Eigen3_FOUND)
    find_path(EIGEN3_INCLUDE_DIR 
        NAMES Eigen/Dense
        PATHS
            /opt/homebrew/Cellar/eigen/*/include/eigen3
            /usr/local/include/eigen3
            /opt/homebrew/include/eigen3
            /usr/include/eigen3
            ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty
    )
    if(EIGEN3_INCLUDE_DIR)
        message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
        include_directories(${EIGEN3_INCLUDE_DIR})
    else()
        # 尝试直接使用Homebrew的Cellar路径
        file(GLOB EIGEN3_CELLAR_DIRS "/opt/homebrew/Cellar/eigen/*/include/eigen3")
        if(EIGEN3_CELLAR_DIRS)
            list(GET EIGEN3_CELLAR_DIRS 0 EIGEN3_INCLUDE_DIR)
            message(STATUS "Found Eigen3 in Cellar: ${EIGEN3_INCLUDE_DIR}")
            include_directories(${EIGEN3_INCLUDE_DIR})
        else()
            message(WARNING "Eigen3 not found, some features may not work")
        endif()
    endif()
endif()

# 添加 algorithm 子目录（算法库）
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../algorithm algorithm)

# 源文件（使用 algorithm 目录下的算法）
set(SOURCES
    planning_test_execute.cpp
    cilqr_adapter_simple.cpp
)

# 创建可执行文件
add_executable(planning_test_execute ${SOURCES})


# 链接 jsoncpp
if(jsoncpp_FOUND)
    target_link_libraries(planning_test_execute jsoncpp_lib)
else()
    target_link_libraries(planning_test_execute ${JSONCPP_LIBRARIES})
endif()

# 链接 algorithm 库
target_link_libraries(planning_test_execute cilqr_algorithm)

# 添加 algorithm 库的头文件路径
target_include_directories(planning_test_execute PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/../animation/include  # 包含 common/ 目录
)

# 添加Eigen3包含路径到target
if(Eigen3_FOUND)
    if(Eigen3_INCLUDE_DIRS)
        target_include_directories(planning_test_execute PUBLIC ${Eigen3_INCLUDE_DIRS})
        message(STATUS "Eigen3: Using Eigen3_INCLUDE_DIRS: ${Eigen3_INCLUDE_DIRS}")
    elseif(EIGEN3_INCLUDE_DIR)
        target_include_directories(planning_test_execute PUBLIC ${EIGEN3_INCLUDE_DIR})
        message(STATUS "Eigen3: Using EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
    else()
        # find_package 找到了但变量未设置，手动查找
        message(STATUS "Eigen3_FOUND is TRUE but no include directory set, searching manually...")
        file(GLOB EIGEN3_CELLAR_DIRS "/opt/homebrew/Cellar/eigen/*/include/eigen3")
        if(EIGEN3_CELLAR_DIRS)
            list(GET EIGEN3_CELLAR_DIRS 0 EIGEN3_INCLUDE_DIR)
            target_include_directories(planning_test_execute PUBLIC ${EIGEN3_INCLUDE_DIR})
            message(STATUS "Eigen3: Using Cellar path: ${EIGEN3_INCLUDE_DIR}")
        elseif(EXISTS /opt/homebrew/include/eigen3)
            target_include_directories(planning_test_execute PUBLIC /opt/homebrew/include/eigen3)
            message(STATUS "Eigen3: Using /opt/homebrew/include/eigen3")
        elseif(EXISTS /usr/local/include/eigen3)
            target_include_directories(planning_test_execute PUBLIC /usr/local/include/eigen3)
            message(STATUS "Eigen3: Using /usr/local/include/eigen3")
        else()
            message(WARNING "Eigen3_FOUND is TRUE but cannot find include directory!")
        endif()
    endif()
    target_compile_definitions(planning_test_execute PRIVATE EIGEN3_FOUND)
    message(STATUS "Eigen3: Found via find_package")
elseif(EIGEN3_INCLUDE_DIR)
    target_include_directories(planning_test_execute PUBLIC ${EIGEN3_INCLUDE_DIR})
    message(STATUS "Eigen3: Using manually found: ${EIGEN3_INCLUDE_DIR}")
else()
    # 尝试直接使用Homebrew的Cellar路径
    file(GLOB EIGEN3_CELLAR_DIRS "/opt/homebrew/Cellar/eigen/*/include/eigen3")
    if(EIGEN3_CELLAR_DIRS)
        list(GET EIGEN3_CELLAR_DIRS 0 EIGEN3_INCLUDE_DIR)
        target_include_directories(planning_test_execute PUBLIC ${EIGEN3_INCLUDE_DIR})
        message(STATUS "Eigen3: Using Cellar path: ${EIGEN3_INCLUDE_DIR}")
    elseif(EXISTS /opt/homebrew/include/eigen3)
        target_include_directories(planning_test_execute PUBLIC /opt/homebrew/include/eigen3)
        message(STATUS "Eigen3: Using /opt/homebrew/include/eigen3")
    elseif(EXISTS /usr/local/include/eigen3)
        target_include_directories(planning_test_execute PUBLIC /usr/local/include/eigen3)
        message(STATUS "Eigen3: Using /usr/local/include/eigen3")
    else()
        message(WARNING "Eigen3 not found, compilation may fail")
    endif()
endif()


target_include_directories(planning_test_execute PRIVATE ${JSONCPP_INCLUDE_DIRS})
message(STATUS "jsoncpp: Using found include dir: ${JSONCPP_INCLUDE_DIRS}")

# 编译选项
target_compile_options(planning_test_execute PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)

# 输出信息
message(STATUS "Building planning_test_execute")
message(STATUS "Output directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
