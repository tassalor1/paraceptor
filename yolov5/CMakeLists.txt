cmake_minimum_required(VERSION 3.10)
project(yolov5)
add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

include_directories(
    /usr/local/cuda/include
    /home/nvidia/TensorRT-8.2.5.1/include/
    ${PROJECT_SOURCE_DIR}/src/
    ${PROJECT_SOURCE_DIR}/plugin/
)

link_directories(
    /usr/local/cuda/lib64
    /home/nvidia/TensorRT-8.2.5.1/lib/
)

file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.cu)
file(GLOB_RECURSE PLUGIN_SRCS ${PROJECT_SOURCE_DIR}/plugin/*.cu)

add_library(myplugins SHARED ${PLUGIN_SRCS})
target_link_libraries(myplugins nvinfer cudart)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Create yolov5_det library
add_library(yolov5_det_lib STATIC
    yolov5_det.cpp
    ${SRCS}
)

target_link_libraries(yolov5_det_lib
    nvinfer
    cudart
    myplugins
    ${OpenCV_LIBS}
)

# Other executables (remove yolov5_det since it's now a library)
add_executable(yolov5_cls yolov5_cls.cpp ${SRCS})
target_link_libraries(yolov5_cls nvinfer cudart myplugins ${OpenCV_LIBS})

add_executable(yolov5_seg yolov5_seg.cpp ${SRCS})
target_link_libraries(yolov5_seg nvinfer cudart myplugins ${OpenCV_LIBS})

# fps_calc executable
add_executable(fps_calc fps_calc.cpp)
target_link_libraries(fps_calc
    yolov5_det_lib
    nvinfer
    cudart
    myplugins
    ${OpenCV_LIBS}
    stdc++fs
    cnpy
)
