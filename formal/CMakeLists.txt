cmake_minimum_required(VERSION 3.20)

project(image_trans_formal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs video videoio calib3d)

add_library(image_trans_core
    src/video_stabilizer.cpp
    src/long_exposure_composer.cpp
)

target_include_directories(image_trans_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(image_trans_core
    PUBLIC
        ${OpenCV_LIBS}
)

target_compile_options(image_trans_core
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

add_executable(image_trans_cli
    src/main.cpp
)

target_link_libraries(image_trans_cli
    PRIVATE
        image_trans_core
)

include(CTest)

if(BUILD_TESTING)
    add_executable(image_trans_tests
        tests/synthetic_tests.cpp
    )

    add_dependencies(image_trans_tests image_trans_cli)

    target_link_libraries(image_trans_tests
        PRIVATE
            image_trans_core
    )

    add_test(NAME image_trans_tests COMMAND image_trans_tests)
endif()
