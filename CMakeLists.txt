cmake_minimum_required(VERSION 3.10)
project(LaneDetection)

# Definir o padrão C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Encontrar pacotes necessários
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# --- Configurar ZeroMQ manualmente (Mantenha esta seção) ---
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

# Configurar CasADi manualmente
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

# Definir caminhos para o TensorRT
set(TensorRT_ROOT "/usr/src/tensorrt")
find_library(TensorRT_LIB nvinfer HINTS ${TensorRT_ROOT}/lib)
find_library(NvInferPlugin_LIB nvinfer_plugin HINTS ${TensorRT_ROOT}/lib)
find_library(NvOnnxParser_LIB nvonnxparser HINTS ${TensorRT_ROOT}/lib)

# Incluir diretórios
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${TensorRT_ROOT}/include
    ${CMAKE_SOURCE_DIR}
    ${CASADI_INCLUDE_DIRS}
    ${ZeroMQ_INCLUDE_DIRS}
    /control_systems/FServo
    /control_systems/BackMotors
    /control_systems/Control
    control_systems/LanesGeometry
    control_systems/MPC
    control_systems/PID
    /MCP2515
    /control_systems
    /object_detection
    /engines
)

# Adicionar executável
add_executable(integrated_vison
    control_systems/ZmqPublisher.cpp
    control_systems/ZmqSubscriber.cpp
    control_systems/utils_control.cpp
    main.cpp
    control_systems/LanesGeometry/lane.cpp
    control_systems/LanesGeometry/mask.cpp
    control_systems/MPC/nmpc.cpp
    control_systems/PID/pid.cpp
    control_systems/PID/SpeedPIDController.cpp
    control_systems/PID/SpeedPIDTuner.cpp
    control_systems/cam.cpp
    control_systems/LanesGeometry/infer.cpp
    control_systems/BackMotors/BackMotors.cpp
    control_systems/Control/ControlAssembly.cpp
    control_systems/Control/Controller.cpp
    control_systems/FServo/FServo.cpp
    # Ficheiros do CAN Bus
    MCP2515/MCP2515Configurator.cpp
    MCP2515/CANMessageProcessor.cpp
    MCP2515/MCP2515Controller.cpp
    MCP2515/SPIController.cpp
    MCP2515/CanBusManager.cpp
)

# Linkar bibliotecas
target_link_libraries(integrated_vison
    ${OpenCV_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${TensorRT_LIB}
    ${NvInferPlugin_LIB}
    ${NvOnnxParser_LIB}
    ${CASADI_LIBRARIES}
    ${ZeroMQ_LIBRARIES}
    # Adicionar bibliotecas necessárias para as threads e SPI
    rt
    pthread
)

# Configurações adicionais para Jetson
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast")

# Mensagens de depuração
message(STATUS "Final CASADI_INCLUDE_DIRS: ${CASADI_INCLUDE_DIRS}")
message(STATUS "Final CASADI_LIBRARIES: ${CASADI_LIBRARIES}")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-fsanitize=address -fsanitize=thread -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address -fsanitize=thread)
endif()

