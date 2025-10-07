cmake_minimum_required(VERSION 3.10)
project(LaneDetection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

find_path(ZeroMQ_INCLUDE_DIRS
    NAMES zmq.hpp
    PATHS /usr/local/include /usr/include
    NO_CMAKE_FIND_ROOT_PATH
)
find_library(ZeroMQ_LIBRARIES
    NAMES zmq
    PATHS /usr/local/lib /usr/lib /usr/lib/aarch64-linux-gnu
    NO_CMAKE_FIND_ROOT_PATH
)
if(NOT ZeroMQ_INCLUDE_DIRS)
    message(FATAL_ERROR "ZeroMQ include (zmq.hpp) nao encontrado. Por favor, instale 'libzmq-dev'.")
endif()
if(NOT ZeroMQ_LIBRARIES)
    message(FATAL_ERROR "ZeroMQ library (libzmq.so) nao encontrada. Por favor, instale 'libzmq-dev'.")
endif()
message(STATUS "ZeroMQ configurado manualmente!")
message(STATUS "ZeroMQ_INCLUDE_DIRS: ${ZeroMQ_INCLUDE_DIRS}")
message(STATUS "ZeroMQ_LIBRARIES: ${ZeroMQ_LIBRARIES}")

set(CASADI_INCLUDE_DIRS /usr/local/include)
set(CASADI_LIBRARIES /usr/local/lib/libcasadi.so)
if(NOT EXISTS "${CASADI_INCLUDE_DIRS}/casadi/casadi.hpp")
    message(FATAL_ERROR "CasADi include não encontrado em ${CASADI_INCLUDE_DIRS}/casadi/casadi.hpp")
endif()
if(NOT EXISTS "${CASADI_LIBRARIES}")
    message(FATAL_ERROR "CasADi library não encontrada em ${CASADI_LIBRARIES}")
endif()
message(STATUS "CasADi configurado manualmente!")
message(STATUS "CASADI_INCLUDE_DIRS: ${CASADI_INCLUDE_DIRS}")
message(STATUS "CASADI_LIBRARIES: ${CASADI_LIBRARIES}")

set(TensorRT_ROOT "/usr/src/tensorrt")
find_library(TensorRT_LIB nvinfer HINTS ${TensorRT_ROOT}/lib)
find_library(NvInferPlugin_LIB nvinfer_plugin HINTS ${TensorRT_ROOT}/lib)
find_library(NvOnnxParser_LIB nvonnxparser HINTS ${TensorRT_ROOT}/lib)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${TensorRT_ROOT}/include
    ${CMAKE_SOURCE_DIR}
    ${CASADI_INCLUDE_DIRS}
    ${ZeroMQ_INCLUDE_DIRS}
    control_systems/LanesGeometry
    control_systems/MPC
    control_systems/PID
    control_systems/Cam
    /control_systems
    /object_detection
    /engines
)

add_executable(integrated_vison
    control_systems/ZMQ/ZmqPublisher.cpp
    control_systems/ZMQ/ZmqSubscriber.cpp
    control_systems/utils_control.cpp
    main.cpp
    utils.cpp
    control_systems/LanesGeometry/lane.cpp
    control_systems/LanesGeometry/mask.cpp
    control_systems/MPC/nmpc.cpp
    control_systems/PID/pid.cpp
    control_systems/Cam/cam.cpp
    control_systems/LanesGeometry/infer.cpp
    object_detection/TensorRTYOLO.cpp
    object_detection/frame.cpp
    object_detection/fps.cpp
)

target_link_libraries(integrated_vison
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${TensorRT_LIB}
    ${NvInferPlugin_LIB}
    ${NvOnnxParser_LIB}
    ${CASADI_LIBRARIES}
    ${ZeroMQ_LIBRARIES}

    rt
    pthread
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast")

message(STATUS "Final CASADI_INCLUDE_DIRS: ${CASADI_INCLUDE_DIRS}")
message(STATUS "Final CASADI_LIBRARIES: ${CASADI_LIBRARIES}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-fsanitize=address -fsanitize=thread -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address -fsanitize=thread)
endif()

