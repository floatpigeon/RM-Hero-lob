cmake_minimum_required(VERSION 3.20)

project(HeroLob LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc videoio)

add_executable(hero_lob
    core/src/main.cpp
    core/src/pipeline.cpp
    core/src/capture.cpp
    core/src/identifier.cpp
    core/src/tracker.cpp
    core/src/reference_frame_selector.cpp
    core/src/image_registrator.cpp
    core/src/background_remover.cpp
    core/src/tracker_processor.cpp
    core/src/image_synthesis.cpp
)

target_include_directories(hero_lob
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

target_link_libraries(hero_lob
    PRIVATE
        ${OpenCV_LIBS}
)
