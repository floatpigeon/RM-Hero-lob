cmake_minimum_required(VERSION 3.18)

project(image_trans LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)

add_library(image_trans_core
    src/simulator.cpp
)

target_include_directories(image_trans_core
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(image_trans_core
    PUBLIC
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        opencv_videoio
)

target_compile_options(image_trans_core
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

add_executable(image_trans
    src/main.cpp
)

target_link_libraries(image_trans
    PRIVATE
        image_trans_core
)

add_executable(image_trans_test
    src/test_main.cpp
)

target_link_libraries(image_trans_test
    PRIVATE
        image_trans_core
)

enable_testing()
add_test(NAME image_trans_core_test COMMAND image_trans_test)
