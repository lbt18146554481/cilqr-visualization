cmake_minimum_required(VERSION 3.10)
project(MotionPlanning)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Configuring Debug build")
    set(CMAKE_BUILD_TYPE "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O0 -g")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "Configuring Release build")
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
endif()

# 设置3rdparty目录路径（统一在根目录）
set(THIRDPARTY_DIR ${CMAKE_CURRENT_LIST_DIR}/../3rdparty)

# 添加matplotlib_cpp库
add_library(matplotlib_cpp INTERFACE)

# 优先查找conda环境的Python
if(DEFINED ENV{CONDA_PREFIX} OR DEFINED Python3_ROOT_DIR)
    if(DEFINED ENV{CONDA_PREFIX})
        set(CONDA_PYTHON_ROOT $ENV{CONDA_PREFIX})
    else()
        set(CONDA_PYTHON_ROOT ${Python3_ROOT_DIR})
    endif()
    # 明确指定conda的Python路径
    set(Python3_ROOT_DIR ${CONDA_PYTHON_ROOT})
    set(Python3_FIND_STRATEGY LOCATION)
    set(Python3_FIND_REGISTRY NEVER)
    # 直接设置Python可执行文件路径
    if(EXISTS ${CONDA_PYTHON_ROOT}/bin/python3)
        set(Python3_EXECUTABLE ${CONDA_PYTHON_ROOT}/bin/python3 CACHE FILEPATH "Python3 executable")
    endif()
    message(STATUS "Using conda Python from: ${CONDA_PYTHON_ROOT}")
endif()

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
message(STATUS "Found Python3: ${Python3_EXECUTABLE}")
message(STATUS "Python3 version: ${Python3_VERSION}")
message(STATUS "Python3 library: ${Python3_LIBRARIES}")
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)

find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE Python3::NumPy)
  message(STATUS "Found NumPy: ${Python3_NumPy_INCLUDE_DIRS}")
else()
  # 尝试手动查找NumPy
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
    OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(NUMPY_INCLUDE_DIR AND EXISTS ${NUMPY_INCLUDE_DIR})
    message(STATUS "Found NumPy manually at: ${NUMPY_INCLUDE_DIR}")
    target_include_directories(matplotlib_cpp INTERFACE ${NUMPY_INCLUDE_DIR})
    # 需要链接NumPy库
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import numpy; import os; print(os.path.join(os.path.dirname(numpy.__file__), 'core', 'lib'))"
      OUTPUT_VARIABLE NUMPY_LIB_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
  else()
    message(WARNING "NumPy not found, Eigen support will be disabled")
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
  endif()
endif()

# 添加fmt库（从源码编译）
add_subdirectory(${THIRDPARTY_DIR}/fmt ${CMAKE_BINARY_DIR}/3rdparty/fmt)
set(FMT_INCLUDE_DIRS ${THIRDPARTY_DIR}/fmt/include)
set(FMT_LIBRARIES fmt::fmt)

# 添加spdlog库（从源码编译）
option(SPDLOG_BUILD_SHARED "Build shared library" OFF)
option(SPDLOG_FMT_EXTERNAL "Use external fmt library" ON)
add_subdirectory(${THIRDPARTY_DIR}/spdlog ${CMAKE_BINARY_DIR}/3rdparty/spdlog)
set(SPDLOG_INCLUDE_DIRS ${THIRDPARTY_DIR}/spdlog/include)
set(SPDLOG_LIBRARIES spdlog::spdlog)

# 添加yaml-cpp库（从源码编译）
option(YAML_CPP_BUILD_TESTS "Build yaml-cpp tests" OFF)
option(YAML_CPP_BUILD_TOOLS "Build yaml-cpp tools" OFF)
option(YAML_CPP_BUILD_CONTRIB "Build yaml-cpp contrib" OFF)
add_subdirectory(${THIRDPARTY_DIR}/yaml-cpp ${CMAKE_BINARY_DIR}/3rdparty/yaml-cpp)
set(YAML_CPP_INCLUDE_DIRS ${THIRDPARTY_DIR}/yaml-cpp/include)
set(YAML_CPP_LIBRARIES yaml-cpp)

# 查找Eigen3库
find_package(Eigen3 QUIET)
# 即使find_package找到了，也要检查路径是否有效
if(NOT EIGEN3_INCLUDE_DIR AND NOT EIGEN3_INCLUDE_DIRS)
    # 如果find_package找不到或路径为空，手动查找
    set(EIGEN3_INCLUDE_DIR "")
    
    # 检查Homebrew的Cellar路径（使用通配符查找最新版本）
    file(GLOB EIGEN3_CELLAR_PATHS "/opt/homebrew/Cellar/eigen/*/include/eigen3")
    if(EIGEN3_CELLAR_PATHS)
        list(GET EIGEN3_CELLAR_PATHS 0 EIGEN3_INCLUDE_DIR)
        message(STATUS "Found Eigen3 in Cellar: ${EIGEN3_INCLUDE_DIR}")
    else()
        # 检查其他常见路径
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

# 处理不同的变量名
if(EIGEN3_INCLUDE_DIRS AND NOT EIGEN3_INCLUDE_DIR)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Eigen3 not found! Please install it: brew install eigen")
else()
    message(STATUS "Using Eigen3 at: ${EIGEN3_INCLUDE_DIR}")
endif()

# 设置包含目录
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_LIST_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${THIRDPARTY_DIR})
include_directories(${FMT_INCLUDE_DIRS})
include_directories(${SPDLOG_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})

# 添加算法子目录
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../algorithm algorithm)

# 收集源文件（排除4D模型相关文件）
file(GLOB SOURCE_FILES 
    "${PROJECT_SOURCE_DIR}/src/motion_planning.cpp"
    "${PROJECT_SOURCE_DIR}/src/cilqr_adapter.cpp"
    "${PROJECT_SOURCE_DIR}/src/cubic_spline.cpp"
    "${PROJECT_SOURCE_DIR}/src/global_config.cpp"
    "${PROJECT_SOURCE_DIR}/src/utils.cpp"
)

# 设置第三方库列表
set(3RDPARTY_LIBS 
    matplotlib_cpp 
    ${FMT_LIBRARIES} 
    ${SPDLOG_LIBRARIES} 
    ${YAML_CPP_LIBRARIES}
    cilqr_algorithm  # 算法库
)

# 添加可执行文件并链接库
add_executable(motion_planning ${SOURCE_FILES})
target_compile_features(motion_planning PUBLIC cxx_std_17)
target_link_libraries(motion_planning ${3RDPARTY_LIBS})

# 添加算法库的头文件路径
target_include_directories(motion_planning PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/../algorithm/include
)

# 安装目标
install(TARGETS motion_planning DESTINATION bin)
